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
local TESTMODE_RTL = 6
local TESTMODE_FINISH = 7

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
local A1_DEFDEG = 20
local A1_DEFZSPEED = 2.5
local A1_CNT_1 = 180
local A1_CNT_2 = A1_CNT_1 + 180 * 2
local A1_CNT_3 = A1_CNT_2 + 180
local A1_CNT_4 = A1_CNT_3 + 20
local A1_SPEED_H = 1.2
local A1_SPEED_V = 0.5

local a1_state = {}
--             start     end       r  phase(deg)
a1_state[1] = {       0, A1_CNT_1, 1, 270}
a1_state[2] = {A1_CNT_1, A1_CNT_2, 2,  90}
a1_state[3] = {A1_CNT_2, A1_CNT_3, 1, 270}
a1_state[4] = {A1_CNT_3, A1_CNT_4, 0,   0}
a1_state[5] = {     nil,      nil, 0,   0}

local a1_mode   = "H"

local A1_START  = 1
local A1_END    = 2
local A1_RADIUS = 3
local A1_PHASE  = 4

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
                    gcs:send_text(0, "[L]DRAW HEART MARK")
                    test_mode = TESTMODE_ACROBAT1
                end
            end
            
        -- *** draw heart mark ***
        elseif (test_mode == TESTMODE_ACROBAT1) then
            local home = ahrs:get_home()
            local curr_loc = ahrs:get_position()
            local vec_from_home = curr_loc:get_distance_NED(home)

            if (a1_mode == "H") then
                a1_speed = A1_SPEED_H
            else
                a1_speed = A1_SPEED_V
            end

            for i, v in ipairs(a1_state) do
                if not (v[A1_END]) then
                    if (a1_mode == "H") then
                        a1_mode = "V"
                        cycle_cnt = 0
                    else
                        cycle_cnt = 0
                        gcs:send_text(0, "[L]ACROBAT")
                        test_mode = TESTMODE_ACROBAT2
                    end
                    break
                end

                if (cycle_cnt * a1_speed <= v[A1_END]) then
                    a1_now_roll   = 0
                    a1_now_pitch  = 0
                    a1_climb_rate = 0
                    if not (v[A1_RADIUS] == 0) then
                        deg = (cycle_cnt * a1_speed - v[A1_START]) / v[A1_RADIUS] + v[A1_PHASE]
                        if (a1_mode == "H") then
                            a1_now_roll   = A1_DEFDEG * (math.cos(math.rad(deg)))
                            a1_now_pitch  = A1_DEFDEG * (math.sin(math.rad(deg)))
                            vehicle:set_target_angle_and_climbrate(a1_now_roll, a1_now_pitch, 0, a1_climb_rate, false, 0)
                        else
                            local target_vel = Vector3f()
                            target_vel:x(A1_DEFZSPEED * (math.cos(math.rad(deg))))
                            target_vel:y(0)
                            target_vel:z(A1_DEFZSPEED * (math.sin(math.rad(deg))))
                            vehicle:set_target_velocity_NED(target_vel)
                        end
                    end
                    break
                end
            end
            
            cycle_cnt = cycle_cnt + 1
            
        -- *** acrobat ***
        elseif (test_mode == TESTMODE_ACROBAT2) then
            -- control
            if (cycle_cnt < 50) then  -- stabilize
                vehicle:set_target_angle_and_climbrate(0, 0, 0, 0, FALSE, 0)
            elseif (cycle_cnt == 50) then  -- Roll left
                gcs:send_text(0, "[L]----------Roll Left----------")
                SRV_Channels:set_output_pwm_chan_timeout( 0, 1600, 300)    -- right    ccw
                SRV_Channels:set_output_pwm_chan_timeout( 1, 1300, 300)    -- left     ccw
                SRV_Channels:set_output_pwm_chan_timeout( 2, 1550, 300)    -- front    cw
                SRV_Channels:set_output_pwm_chan_timeout( 3, 1550, 300)    -- rear     cw
            elseif (cycle_cnt == 90) then  -- Roll right
                gcs:send_text(0, "[L]----------Roll Right----------")
                SRV_Channels:set_output_pwm_chan_timeout( 0, 1300, 300)    -- right    ccw
                SRV_Channels:set_output_pwm_chan_timeout( 1, 1600, 300)    -- left     ccw
                SRV_Channels:set_output_pwm_chan_timeout( 2, 1550, 300)    -- front    cw
                SRV_Channels:set_output_pwm_chan_timeout( 3, 1550, 300)    -- rear     cw
            elseif (cycle_cnt == 130) then
                gcs:send_text(0, "[L]----------Backflip----------")
                SRV_Channels:set_output_pwm_chan_timeout( 0, 1550, 300)    -- right    ccw
                SRV_Channels:set_output_pwm_chan_timeout( 1, 1550, 300)    -- left     ccw
                SRV_Channels:set_output_pwm_chan_timeout( 2, 1600, 300)    -- front    cw
                SRV_Channels:set_output_pwm_chan_timeout( 3, 1300, 300)    -- rear     cw
            elseif (cycle_cnt == 170) then
                gcs:send_text(0, "[L]----------Forward Rotation----------")
                SRV_Channels:set_output_pwm_chan_timeout( 0, 1550, 300)    -- right    ccw
                SRV_Channels:set_output_pwm_chan_timeout( 1, 1550, 300)    -- left     ccw
                SRV_Channels:set_output_pwm_chan_timeout( 2, 1300, 300)    -- front    cw
                SRV_Channels:set_output_pwm_chan_timeout( 3, 1600, 300)    -- rear     cw
            elseif (cycle_cnt > 210) then
                cycle_cnt = 0
                gcs:send_text(0, "[L]SWOOP")
                test_mode = TESTMODE_ACROBAT3
            end
            
            cycle_cnt = cycle_cnt + 1
            
        -- *** swoop ***
        elseif (test_mode == TESTMODE_ACROBAT3) then
            if (cycle_cnt < 50) then  -- stabilize
                vehicle:set_target_angle_and_climbrate(0, 0, 0, 0, FALSE, 0)
            else
                local home = ahrs:get_home()
                local curr_loc = ahrs:get_position()
                if home and curr_loc then
                    local vec_from_home = home:get_distance_NED(curr_loc)
                    if (math.abs(20 + vec_from_home:z()) < 1) then
                        vehicle:set_target_angle_and_climbrate(0, 0, 0, 0, FALSE, 0)
                        
                        cycle_cnt = 0
                        gcs:send_text(0, "[L]RTL")
                        test_mode = TESTMODE_RTL
                    else
                        gcs:send_text(0, "[L]stop moter output")
                        SRV_Channels:set_output_pwm_chan_timeout( 0, 0, 300)
                        SRV_Channels:set_output_pwm_chan_timeout( 1, 0, 300)
                        SRV_Channels:set_output_pwm_chan_timeout( 2, 0, 300)
                        SRV_Channels:set_output_pwm_chan_timeout( 3, 0, 300)
                    end
                end
            end
            
            cycle_cnt = cycle_cnt + 1
            
        -- *** RTL ***
        elseif (test_mode == TESTMODE_RTL) then
            vehicle:set_mode(6)
            test_mode = TESTMODE_FINISH
        end
    end

    -- print debug
    -- if (test_mode >= TESTMODE_ACROBAT1) then
    --     now_yaw = math.deg(ahrs:get_yaw())
    --     now_roll = math.deg(ahrs:get_roll())
    --     now_pitch = math.deg(ahrs:get_pitch())
    --     gcs:send_text(0, string.format("[L]now r:%.1f, p:%.1f, y:%.1f", now_roll, now_pitch, now_yaw))
    -- end
    
    return update, 100
end

return update()
