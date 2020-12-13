-- **************** memo ****************
-- flight mode : FLTMODES[] STAB:0 ACRO:1 GUIDED:4  RTL:6

-- **************** const data ****************
local TEST_ALTITUDE = 100

-- test mode
local TESTMODE_CHANGE_GUIDED_MODE = 0
local TESTMODE_TAKE_OFF = 1
local TESTMODE_CLIMB_START_POSITON = 2
local TESTMODE_ACROBAT1 = 3
local TESTMODE_ACROBAT2 = 4
local TESTMODE_ACROBAT3 = 5
local TESTMODE_ACROBAT3 = 6
local TESTMODE_RTL = 7
local TESTMODE_FINISH = 8

-- **************** common RAM ****************
local test_mode = 0
local cycle_cnt = 0

-- angle
local target_yaw = 0
local target_roll = 0
local target_pitch = 0
local climb_rate = 0 -- m/s
local use_yaw_rate = false
local yaw_rate = 0 -- degs/s

-- output for debug
local now_yaw = 0
local now_roll = 0
local now_pitch = 0

-- **************** RAM part of acrobat ****************
-- *** acrobat1 ***

-- *** acrobat2 ***

-- *** acrobat3 ***

-- *** acrobat4 ***

-- **************** script ****************
function update()
    if not arming:is_armed() then                               -- reset state when disarmed
        test_mode = TESTMODE_CHANGE_GUIDED_MODE
        
    else
        -- *** change to Guided mode ***
        if (test_mode == TESTMODE_CHANGE_GUIDED_MODE) then
            if (vehicle:set_mode(4)) then
                gcs:send_text(0, "[L]change Guided mode")
                test_mode = TESTMODE_TAKE_OFF
            end
            
        -- *** takeoff ***
        elseif (test_mode == TESTMODE_TAKE_OFF) then
            if (vehicle:start_takeoff(TEST_ALTITUDE)) then
                gcs:send_text(0, "[L]takeoff")
                test_mode = TESTMODE_CLIMB_START_POSITON
            end
            
        -- *** crimb start position ***
        elseif (test_mode == TESTMODE_CLIMB_START_POSITON) then
            local home = ahrs:get_home()
            local curr_loc = ahrs:get_position()
            if home and curr_loc then
                local vec_from_home = home:get_distance_NED(curr_loc)
                if (math.abs(TEST_ALTITUDE + vec_from_home:z()) < 1) then
                    gcs:send_text(0, "[L]start acrobat1")
                    test_mode = TESTMODE_ACROBAT1
                end
            end
            
        -- *** tachibana ***
        elseif (test_mode == TESTMODE_ACROBAT1) then
            -- check cycle counter
            cycle_cnt = cycle_cnt + 1
            if (cycle_cnt > 1) then
                cycle_cnt = 0
                gcs:send_text(0, "[L]start acrobat2")
                test_mode = TESTMODE_ACROBAT2
            end
            
        -- *** ono ***
        elseif (test_mode == TESTMODE_ACROBAT2) then
            -- control
            -- vehicle:set_target_angle_and_climbrate(target_roll, target_pitch, target_yaw, climb_rate, use_yaw_rate, yaw_rate)
            -- SRV_Channels:set_output_pwm_chan_timeout( 0, 1530, 1000)
            -- SRV_Channels:set_output_pwm_chan_timeout( 1, 1535, 1000)
            -- SRV_Channels:set_output_pwm_chan_timeout( 2, 1535, 1000)
            -- SRV_Channels:set_output_pwm_chan_timeout( 3, 1530, 1000)
            
            -- print debug
            -- now_yaw = math.deg(ahrs:get_yaw())
            -- now_roll = math.deg(ahrs:get_roll())
            -- now_pitch = math.deg(ahrs:get_pitch())
            -- gcs:send_text(0, string.format("[L]now r:%.1f, p:%.1f, y:%.1f", now_roll, now_pitch, now_yaw))
            
            -- check cycle counter
            cycle_cnt = cycle_cnt + 1
            if (cycle_cnt > 1) then
                cycle_cnt = 0
                gcs:send_text(0, "[L]start acrobat3")
                test_mode = TESTMODE_ACROBAT3
            end
            
        -- *** suzuki ***
        elseif (test_mode == TESTMODE_ACROBAT3) then
            -- check cycle counter
            cycle_cnt = cycle_cnt + 1
            if (cycle_cnt > 1) then
                cycle_cnt = 0
                gcs:send_text(0, "[L]start acrobat4")
                test_mode = TESTMODE_ACROBAT4
            end
            
        -- *** shibuya ***
        elseif (test_mode == TESTMODE_ACROBAT4) then
            -- check cycle counter
            cycle_cnt = cycle_cnt + 1
            if (cycle_cnt > 1) then
                cycle_cnt = 0
                test_mode = TESTMODE_RTL
            end
            
        -- *** RTL ***
        elseif (test_mode == TESTMODE_RTL) then
            vehicle:set_mode(6)
            test_mode = TESTMODE_FINISH
            gcs:send_text(0, "[L]RTL")
        end
    end

    return update, 100
end

return update()
