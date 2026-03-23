	setLuaScript( R"(
AIRBAG = 0x050
TCU_1088_440 = 0x440
TCU_1344_540 = 0x540
-- 1440
BRAKE_2 = 0x5A0

VWTP_OUT = 0x200
VWTP_IN = 0x202
VWTP_TESTER = 0x300

-- 640
MOTOR_1 = 0x280
-- 644
MOTOR_BRE = 0x284
-- 648
MOTOR_2 = 0x288
-- 800
Kombi_1 = 0x320
-- 896
MOTOR_3 = 0x380
-- 1152
MOTOR_5 = 0x480
-- 1160
MOTOR_6 = 0x488
-- 1386
ACC_GRA = 0x56A
-- 1408 the one with variable payload
MOTOR_INFO = 0x580
-- 1416
MOTOR_7 = 0x588

TCU_BUS = 1

fakeTorque = 0

-- ===========================================================================
-- VEHICLE-SPECIFIC CALIBRATION — adjust to your DSG DQ250 build
-- ===========================================================================
-- DQ250 6-speed gear ratios (ratio = engine_rpm / driveshaft_rpm in each gear)
local GEAR_RATIOS    = { 3.500, 1.960, 1.320, 0.974, 0.771, 0.609 }
local FINAL_DRIVE    = 3.647    -- rear/front differential final drive ratio
local WHEEL_REV_PER_KM = 489.0  -- wheel revolutions per km (e.g. 205/55R16)

-- ===========================================================================
-- RPM MATCH (DOWNSHIFT ETB BLIP) CALIBRATION
-- ===========================================================================
local RPM_MATCH_MAX_ETB_ADD     = 15.0   -- max throttle % added during blip
local RPM_MATCH_PEDAL_THRESHOLD =  8.0   -- % pedal: no blip if driver is on gas
local RPM_MATCH_RPM_DELTA_MIN   = 150    -- RPM: no blip if delta smaller than this
local RPM_MATCH_TIMEOUT_MS      = 1500   -- ms: abort if RPM not matched in time
local RPM_MATCH_RAMP_RATE       =  2.5   -- % ETB added per onTick call (~300 Hz)
local RPM_MATCH_RPM_TOLERANCE   =  80    -- RPM: success band around target

-- ===========================================================================
-- SHIFT STATE MACHINE
-- All shift state is in a single table to avoid global variable pollution
-- and to make the logic explicit and collision-free.
-- ===========================================================================
local shiftState = {
    phase     = "IDLE",  -- "IDLE" / "UPSHIFT" / "DOWNSHIFT"
    prevActive = 0,
    timer     = Timer.new(),
    TIMEOUT_MS = 3000,   -- safety: release overrides if TCU never signals completion
}

-- RPM match sub-state (active only during DOWNSHIFT phase)
local rpmMatch = {
    phase     = "IDLE",  -- "IDLE"/"DECIDING"/"BLIPPING"/"HOLDING"/"REMOVING"/"DONE"
    targetRpm = 0,
    etbAdd    = 0,
    timer     = Timer.new(),
}

hexstr = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, "A", "B", "C", "D", "E", "F" }

function toHexString(num)
	if num == 0 then
		return '0'
	end

	local result = ""
	while num > 0 do
		local n = num % 16
		result = hexstr[n + 1] ..result
		num = math.floor(num / 16)
	end
	return result
end

function arrayToString(arr)
	local str = ""
	local index = 1
	while arr[index] ~= nil do
		str = str.." "..toHexString(arr[index])
		index = index + 1
	end
	return str
end

function onTcu2(bus, id, dlc, data)
--	print("onTcu2")
end

-- FIX: all intermediate variables are local to avoid global scope pollution
-- and to make recursive calls safe (bitsToHandleNow, mask, etc.)
function setBitRange(data, totalBitIndex, bitWidth, value)
	local byteIndex = totalBitIndex >> 3
	local bitInByteIndex = totalBitIndex - byteIndex * 8
	if (bitInByteIndex + bitWidth > 8) then
		local bitsToHandleNow = 8 - bitInByteIndex
		setBitRange(data, totalBitIndex + bitsToHandleNow, bitWidth - bitsToHandleNow, value >> bitsToHandleNow)
		bitWidth = bitsToHandleNow
	end
	local mask = (1 << bitWidth) - 1
	data[1 + byteIndex] = data[1 + byteIndex] & (~(mask << bitInByteIndex))
	local maskedValue = value & mask
	local shiftedValue = maskedValue << bitInByteIndex
	data[1 + byteIndex] = data[1 + byteIndex] | shiftedValue
end

function setTwoBytes(data, offset, value)
	data[offset + 1] = value % 255
	data[offset + 2] = (value >> 8) % 255
end

shallSleep = Timer.new()

-- we want to turn on with hardware switch while ignition key is off
hadIgnitionEvent = false

function onAirBag(bus, id, dlc, data)
	-- looks like we have ignition key do not sleep!
	shallSleep : reset()
	hadIgnitionEvent = true
end

function xorChecksum(data, targetIndex)
	local index = 1
	local result = 0
	while data[index] ~= nil do
		if index ~= targetIndex then
			result = result ~ data[index]
		end
		index = index + 1
	end
	data[targetIndex] = result
	return result
end

-- FIX: all variables are local
function getBitRange(data, bitIndex, bitWidth)
	local byteIndex = bitIndex >> 3
	local shift = bitIndex - byteIndex * 8
	local value = data[1 + byteIndex]
	if (shift + bitWidth > 8) then
		value = value + data[2 + byteIndex] * 256
	end
	local mask = (1 << bitWidth) - 1
	return (value >> shift) & mask
end

-- ---------------------------------------------------------------------------
-- computeTargetRpm: RPM the engine should reach before downshift clutch engages.
-- targetGear: 1-based index into GEAR_RATIOS table.
-- ---------------------------------------------------------------------------
local function computeTargetRpm(targetGear)
	local vssKph = getSensor("VehicleSpeed") or 0
	if vssKph < 5 then return 0 end
	local ratio = GEAR_RATIOS[targetGear]
	if ratio == nil then return 0 end
	-- wheelRpm = vssKph * WHEEL_REV_PER_KM / 60
	-- driveshaftRpm = wheelRpm * FINAL_DRIVE
	-- engineRpm = driveshaftRpm * gearRatio
	local wheelRpm = vssKph * WHEEL_REV_PER_KM / 60.0
	return wheelRpm * FINAL_DRIVE * ratio
end

-- ---------------------------------------------------------------------------
-- abortShift: release all engine overrides immediately.
-- Called on shift completion, timeout, or abort conditions.
-- ---------------------------------------------------------------------------
local function abortShift()
	setTorqueReductionState(false)
	cancelRpmMatch()
	setEtbAdd(0)
	rpmMatch.phase  = "IDLE"
	rpmMatch.etbAdd = 0
	shiftState.phase = "IDLE"
end

-- ---------------------------------------------------------------------------
-- rpmMatchTick: ETB blip state machine — called every onTick (~300 Hz).
-- Only active during DOWNSHIFT phase.
-- ---------------------------------------------------------------------------
local function rpmMatchTick()
	if rpmMatch.phase == "IDLE" then return end

	local currentRpm = getSensor("Rpm") or 0
	local pedal      = getSensor("AcceleratorPedalPrimary") or getSensor("Tps1") or 0
	local error      = rpmMatch.targetRpm - currentRpm

	-- ABORT: driver pressed accelerator or global timeout
	if pedal > RPM_MATCH_PEDAL_THRESHOLD
		or rpmMatch.timer:getElapsedMs() > RPM_MATCH_TIMEOUT_MS then
		setEtbAdd(0)
		cancelRpmMatch()
		rpmMatch.phase  = "IDLE"
		rpmMatch.etbAdd = 0
		return
	end

	if rpmMatch.phase == "DECIDING" then
		if error < RPM_MATCH_RPM_DELTA_MIN then
			-- RPM already close enough — skip blip, go straight to done
			rpmMatch.phase = "DONE"
		else
			rpmMatch.phase = "BLIPPING"
		end

	elseif rpmMatch.phase == "BLIPPING" then
		-- Proportional open: more ETB when further from target, capped at max
		local targetEtb = RPM_MATCH_MAX_ETB_ADD * math.min(error / 500.0, 1.0)
		targetEtb = math.max(targetEtb, 0)
		rpmMatch.etbAdd = math.min(rpmMatch.etbAdd + RPM_MATCH_RAMP_RATE, targetEtb)
		setEtbAdd(rpmMatch.etbAdd)
		-- Also inform firmware RPM match module for live data / advance boost
		setRpmMatchTarget(rpmMatch.targetRpm)
		if error < RPM_MATCH_RPM_TOLERANCE then
			rpmMatch.phase = "HOLDING"
		end

	elseif rpmMatch.phase == "HOLDING" then
		-- Maintain ETB while RPM stabilizes; firmware confirms via getRpmMatchState()
		if getRpmMatchState() == 2 or error < RPM_MATCH_RPM_TOLERANCE then
			rpmMatch.phase = "REMOVING"
		elseif error > RPM_MATCH_RPM_TOLERANCE * 2 then
			-- RPM dropped back, keep blipping
			rpmMatch.phase = "BLIPPING"
		end

	elseif rpmMatch.phase == "REMOVING" then
		-- Ramp down ETB smoothly to avoid torque spike on clutch engagement
		rpmMatch.etbAdd = math.max(rpmMatch.etbAdd - RPM_MATCH_RAMP_RATE, 0)
		setEtbAdd(rpmMatch.etbAdd)
		if rpmMatch.etbAdd <= 0 then
			rpmMatch.phase = "DONE"
		end
	end
	-- "DONE": stay here until abortShift() resets state (shift complete signal)
end

-- ---------------------------------------------------------------------------
-- onTcu1: CAN callback for TCU gear shift requests (0x440)
-- FIX: all local variables — no global state pollution from this callback.
-- FIX: elseif prevents simultaneous upshift+downshift activation.
-- FIX: safety timeout so we don't stay stuck if TCU never sends shift_complete.
-- ---------------------------------------------------------------------------
local counter440 = 0

function onTcu1(bus, id, dlc, data)
	-- Use local variables — CAN callbacks run asynchronously
	local isShiftActive    = getBitRange(data, 0, 1)
	local upshiftRequest   = getBitRange(data, 4, 1)
	local downshiftRequest = getBitRange(data, 5, 1)

	-- ---- Rising edge: new shift request ----
	if shiftState.prevActive == 0 and isShiftActive == 1 then
		shiftState.timer:reset()

		-- FIX: elseif — upshift has priority; both cannot activate simultaneously
		if upshiftRequest == 1 then
			-- UPSHIFT: firmware torque reduction handles timing retard + spark skip
			setTorqueReductionState(true)
			shiftState.phase = "UPSHIFT"

		elseif downshiftRequest == 1 then
			-- DOWNSHIFT: compute RPM target and start ETB blip state machine
			local currentGear = math.floor(getSensor("DetectedGear") or 0)
			local targetGear  = currentGear - 1
			if targetGear >= 1 then
				local targetRpm = computeTargetRpm(targetGear)
				if targetRpm > 500 then
					rpmMatch.targetRpm = targetRpm
					rpmMatch.etbAdd    = 0
					rpmMatch.phase     = "DECIDING"
					rpmMatch.timer:reset()
					shiftState.phase   = "DOWNSHIFT"
				end
			end
		end
	end

	-- ---- Falling edge: shift completed ----
	if shiftState.prevActive == 1 and isShiftActive == 0 then
		abortShift()
	end

	shiftState.prevActive = isShiftActive

	-- ---- Safety timeout: if TCU never signals completion ----
	if shiftState.phase ~= "IDLE"
		and shiftState.timer:getElapsedMs() > shiftState.TIMEOUT_MS then
		abortShift()
	end

	counter440 = counter440 + 1
	if counter440 % 50 == 0 then
		print("TCU active=" .. isShiftActive
			.. " up=" .. upshiftRequest
			.. " dn=" .. downshiftRequest
			.. " phase=" .. shiftState.phase)
	end
end

motor1Data   = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
motorBreData = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
motor2Data   = { 0x8A, 0x8D, 0x10, 0x04, 0x00, 0x4C, 0xDC, 0x87 }
motor2mux = {0x8A, 0xE8, 0x2C, 0x64}
canMotorInfo = { 0x00, 0x00, 0x00, 0x14, 0x1C, 0x93, 0x48, 0x14 }
canMotorInfo1= { 0x99, 0x14, 0x00, 0x7F, 0x00, 0xF0, 0x47, 0x01 }
canMotorInfo3= { 0x9B, 0x14, 0x00, 0x11, 0x1F, 0xE0, 0x0C, 0x46 }
canMotor3    = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
motor5mux = {0x1c, 0x54, 0x84, 0xc2}
motor5Data   = { 0x1C, 0x08, 0xF3, 0x55, 0x19, 0x00, 0x00, 0xAD }
motor6Data   = { 0x00, 0x00, 0x00, 0x7E, 0xFE, 0xFF, 0xFF, 0x00 }
motor7Data   = { 0x1A, 0x66, 0x7E, 0x00, 0x00, 0x00, 0x00, 0x00 }
accGraData   = { 0x00, 0x00, 0x08, 0x00, 0x1A, 0x00, 0x02, 0x01 }

setTickRate(300)

everySecondTimer = Timer.new()

mafSensor = Sensor.new("maf")
mafCalibrationIndex = findCurveIndex("mafcurve")

canMotorInfoTotalCounter = 0

canRxAdd(AIRBAG, onAirBag)
canRxAdd(TCU_1088_440, onTcu1)
canRxAdd(TCU_1344_540, onTcu2)
--canRxAdd(BRAKE_2)

rpm = 0

function sendMotor1()
	-- Use firmware real-time torque estimation (VE + MAP + IAT correction).
	-- Falls back to TPS-based estimate when reference params are not configured.
	local estimatedTorque = getInstantTorque()
	if estimatedTorque < 1 then
		-- Fallback: simple TPS-linear estimate while reference params are tuned in.
		estimatedTorque = interpolate(0, 6, 100, 60, tps)
	end

	fakeTorque = estimatedTorque

	-- engineTorque: output shaft torque (after accessories, ~90% of indicated)
	local engineTorque       = estimatedTorque * 0.9
	-- innerTorqWithoutExt: indicated engine torque before drivetrain losses
	local innerTorqWithoutExt = estimatedTorque
	-- torqueLoss: friction + pumping + accessory losses (fixed 20 Nm, tune as needed)
	local torqueLoss          = 20
	-- requestedTorque: driver demand (matches inner torque here)
	local requestedTorque     = estimatedTorque

	motor1Data[2] = math.floor(engineTorque / 0.39)
	setTwoBytes(motor1Data, 2, rpm / 0.25)
	motor1Data[5] = math.floor(innerTorqWithoutExt / 0.4)
	motor1Data[6] = math.floor(tps / 0.4)
	motor1Data[7] = math.floor(torqueLoss / 0.39)
	motor1Data[8] = math.floor(requestedTorque / 0.39)

	txCan(TCU_BUS, MOTOR_1, 0, motor1Data)
end


function onMotor1(bus, id, dlc, data)
	rpm = math.floor(getSensor("Rpm") + 0.5)
	tps = getSensor("Tps1") or 0
	sendMotor1()
end

function sendMotor3()
	local iat = getSensor("Iat") or 0
	local tps = getSensor("Tps1") or 0

	-- desired_wheel_torque: what the TCU uses for shift load calculation.
	-- During RPM match (downshift): report reduced torque once RPM is matched
	-- so the TCU knows it's safe to engage the clutch pack.
	local desired_wheel_torque = fakeTorque
	if shiftState.phase == "DOWNSHIFT"
		and (rpmMatch.phase == "HOLDING" or rpmMatch.phase == "REMOVING" or rpmMatch.phase == "DONE") then
		desired_wheel_torque = fakeTorque * 0.3
	end

	canMotor3[2] = math.floor((iat + 48) / 0.75)
	canMotor3[3] = math.floor(tps / 0.4)
	canMotor3[5] = 0x20
	setBitRange(canMotor3, 24, 12, math.floor(desired_wheel_torque / 0.39))
	canMotor3[8] = math.floor(tps / 0.4)
	txCan(TCU_BUS, MOTOR_3, 0, canMotor3)
end


motorBreCounter = 0
function sendMotorBre()
	motorBreCounter = (motorBreCounter + 1) % 16

	setBitRange(motorBreData, 8, 4, motorBreCounter)
	xorChecksum(motorBreData, 1)

	txCan(TCU_BUS, MOTOR_BRE, 0, motorBreData)
end

motor2counter = 0
function sendMotor2()
	motor2counter = (motor2counter + 1) % 16

	local minTorque = fakeTorque / 2
	motor2Data[7] = math.floor(minTorque / 0.39)

	local brakeBit = rpm < 2000 and 1 or 0
	setBitRange(motor2Data, 16, 1, brakeBit)

	local index = math.floor(motor2counter / 4)
	motor2Data[1] = motor2mux[1 + index]

	txCan(TCU_BUS, MOTOR_2, 0, motor2Data)
end

motor5counter = 0
motor5FuelCounter = 0
function sendMotor5()
    motor5counter = (motor5counter + 1) % 16
	local index = math.floor(motor5counter / 4)
	motor5Data[1] = motor5mux[1 + index]

	xorChecksum(motor5Data, 8)
	txCan(TCU_BUS, MOTOR_5, 0, motor5Data)
end

motor6counter = 0
function sendMotor6()
	motor6counter = (motor6counter + 1) % 16

	local engineTorque  = fakeTorque * 0.9
	local actualTorque  = fakeTorque
	local feedbackGearbox = 255

	motor6Data[2] = math.floor(engineTorque / 0.39)
	motor6Data[3] = math.floor(actualTorque / 0.39)
	motor6Data[6] = math.floor(feedbackGearbox / 0.39)
	setBitRange(motor6Data, 60, 4, motor6counter)

	xorChecksum(motor6Data, 1)
	txCan(TCU_BUS, MOTOR_6, 0, motor6Data)
end

accGraCounter = 0
function sendAccGra()
	accGraCounter = (accGraCounter + 1) % 16
	setBitRange(accGraData, 60, 4, accGraCounter)
	xorChecksum(accGraData, 1)

	txCan(TCU_BUS, ACC_GRA, 0, accGraData)
end

canMotorInfoCounter = 0
function sendMotorInfo()
	canMotorInfoTotalCounter = canMotorInfoTotalCounter + 1
	canMotorInfoCounter = (canMotorInfoCounter + 1) % 16

	local baseByte = canMotorInfoTotalCounter < 6 and 0x80 or 0x90
	canMotorInfo[1]  = baseByte + canMotorInfoCounter
	canMotorInfo1[1] = baseByte + canMotorInfoCounter
	canMotorInfo3[1] = baseByte + canMotorInfoCounter
	local mod4 = canMotorInfoCounter % 4

	if (mod4 == 0 or mod4 == 2) then
		txCan(TCU_BUS, MOTOR_INFO, 0, canMotorInfo)
	elseif (mod4 == 1) then
		txCan(TCU_BUS, MOTOR_INFO, 0, canMotorInfo1)
	else
		txCan(TCU_BUS, MOTOR_INFO, 0, canMotorInfo3)
	end
end

function sendMotor7()
	txCan(TCU_BUS, MOTOR_7, 0, motor7Data)
end

local tcuId = 0

function onCanHello(bus, id, dlc, data)
	-- here we handle 201 packets
	print('Got Hello Response ' ..arrayToString(data))
	tcuId = data[6] * 256 + data[5]

	print('From TCU ' ..tcuId)
	txCan(1, tcuId, 0, { 0xA0, 0x0F, 0x8A, 0xFF, 0x32, 0xFF })
end

local sendCounter = 2
local packetCounter = 1
local payLoadIndex = 0

local groups = { 10 }
-- todo: smarter array size calculation?
local groupsSize = 1

local groupIndex = 1

vssSensor = Sensor.new("VehicleSpeed")
vssSensor : setTimeout(2000)

function onKombi(bus, id, dlc, data)
	speed = getBitRange(data, 46, 10) * 0.32
	vssSensor : set(speed)
end

canRxAdd(Kombi_1, onKombi)

-- unused method did we mean to reset codes? todo: probably remove soon
function onCanTester(bus, id, dlc, data)
	-- here we handle 300 packets

	-- 	print('Got from tester ' ..arrayToString(data))

	if data[1] == 0xA3 then
		-- 		print ("Keep-alive")
		txCan(1, tcuId, 0, { 0xA1, 0x0F, 0x8A, 0xFF, 0x4A, 0xFF })

		groupIndex = groupIndex + 1
		if groupIndex > groupsSize then
			groupIndex = 1
		end
		groupId = groups[groupIndex]
		print (groupIndex .." " ..groupId)


		reqFirst = 0x10 + sendCounter
		print("Requesting next group " ..groupId .." with counter " ..sendCounter)
		txCan(1, tcuId, 0, { reqFirst, 0x00, 0x02, 0x21, groupId })

		sendCounter = sendCounter + 1
		if sendCounter == 16 then
			sendCounter = 0
		end
		return
	end


	if data[1] == 0xA1 then
		print ("Happy 300 packet")
		txCan(1, tcuId, 0, { 0x10, 0x00, 0x02, 0x10, 0x89 })
		return
	end

	if data[1] == 0xA8 then
		print ("They said Bye-Bye")
		return
	end


	if data[1] == 0x10 and dlc == 5 then
		ackPacket = 0xB0 + packetCounter
		print ("Sending ACK B1 " ..ackPacket)
		txCan(1, tcuId, 0, { ackPacket })
		-- request first group from array
		txCan(1, tcuId, 0, { 0x11, 0x00, 0x02, 0x21, groups[1] })
		return
	end

	top4 = math.floor(data[1] / 16)

	if top4 == 0xB then
		-- 		print("Got ACK")
		return
	end

	if top4 == 2 or top4 == 1 then
		print ("Looks like payload index " ..payLoadIndex ..": " ..arrayToString(data))

		if groupId == 2 and payLoadIndex == 0 then
			L7 = data[7]
			H9 = data[8]
			V = 256 * H9 + L7
			print("V 0 " ..V)
		end

		if groupId == 2 and payLoadIndex == 1 then
			L3 = data[3]
			H4 = data[4]
			V = 256 * H4 + L3
			print("V 1 " ..V)
		end

		if groupId == 2 and payLoadIndex == 2 then
			L2 = data[2]
			H3 = data[3]
			V = 256 * H3 + L2
			print("V 2 " ..V)
		end

		payLoadIndex = payLoadIndex + 1

		packetCounter = packetCounter + 1
		if packetCounter > 15 then
			packetCounter = 0
		end

		if top4 == 1 then
			ackPacket = 0xB0 + packetCounter
			print ("Sending payload ACK " ..ackPacket)
			txCan(1, tcuId, 0, { ackPacket })
			payLoadIndex = 0
		end

		return
	end

	print('Got unexpected ' ..arrayToString(data))
end

canRxAdd(VWTP_IN, onCanHello)

--txCan(1, VWTP_OUT, 0, { 0x02, 0xC0, 0x00, 0x10, 0x00, 0x03, 0x01 })

function onTick()
	local freqValue = getSensor("AuxSpeed1") or 0
	local mafValue  = curve(mafCalibrationIndex, 5)
	mafSensor : set(mafValue)

	rpm  = getSensor("Rpm") or 0
	local vbat = getSensor("BatteryVoltage") or 0

	if rpm == 0 then
		canMotorInfoTotalCounter = 0
	end

	-- Run RPM match blip state machine every tick
	if shiftState.phase == "DOWNSHIFT" then
		rpmMatchTick()
	end

	onMotor1(0, 0, 0, nil)
	sendMotor3()

	sendMotor2()
	sendMotor5()
	sendMotor6()
	sendMotor7()
	sendMotorBre()
	sendAccGra()

	local timeToTurnOff = shallSleep : getElapsedSeconds() > 2
	local connectedToUsb = vbat < 4

	if hadIgnitionEvent and timeToTurnOff then
		-- looks like ignition key was removed
		-- 		mcu_standby()
	end

	if everySecondTimer : getElapsedSeconds() > 1 then
		everySecondTimer : reset()

		print("CAN OK " .. getOutput("canWriteOk") .. " not OK " .. getOutput("canWriteNotOk"))

        if rpm > 0 then
		    motor5FuelCounter = motor5FuelCounter + 20
        end

		sendMotorInfo()

	end
end

)");
