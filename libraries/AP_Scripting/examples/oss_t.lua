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
local fast_cycle_cnt = 0

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
local A1_CNT_4 = A1_CNT_3 + 70
local A1_SPEED_H = 1.4
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

-- *** acrobat4 ***
local bf_rate = {0,0,0}
local bf_rate_target = {0,0,0} 
local bf_rate_p_er = {0,0,0}
local bf_rate_p_er_pre = {0,0,0}
local bf_rate_i_er = {0,0,0}
local bf_rate_i_er_lim = {1500.0,1500.0,1500.0}
local bf_rate_d_er = {0,0,0}

local bf_rate_gain_p = {1.0,1.0,1.0}
local bf_rate_gain_i = {0.01, 0.01, 0.0}
local bf_rate_gain_d = {0,0,0}

local bf_rate_pid_out = {0,0,0}

local pwm_max = 2000
local pwm_min = 1000

local roll_scale = 4
local pitch_scale = 4
local yaw_scale = 6

local pwm_out = {0,0,0,0}
local omega_target_deg = {0,0,0}

local q_target = {0,0,0,0}
local q_target_con = {0,0,0,0}
local q_body = {0,0,0,0}
local q_body_con = {0,0,0,0}

local e_target = {0,0,0}
local e_body = {0,0,0}

local q_def = {0,0,0,0}
local e_def = {0,0,0}

local q_ctl_out = {0,0,0,0}

-- *************** functions **************

function get_bf_rate_pid_controller(x_target,y_target,z_target)
    bf_rate_target = {x_target,y_target,z_target}
    rates = ahrs:get_gyro()
    if rates then
        roll_rate = math.deg(rates:x())
        pitch_rate = math.deg(rates:y())
        yaw_rate = math.deg(rates:z())
    else
        roll_rate = 0
        pitch_rate = 0
        yaw_rate = 0
    end
    bf_rate[1] = roll_rate
    bf_rate[2] = pitch_rate
    bf_rate[3] = yaw_rate
    --gcs:send_text(0, string.format("[L]now x:%.1f, y:%.1f, z:%.1f", bf_rate[1], bf_rate[2], bf_rate[3]))
    for j = 1,3 do
        bf_rate_p_er[j] = bf_rate_target[j] - bf_rate[j]
        bf_rate_i_er[j] = bf_rate_i_er[j] + bf_rate_p_er[j]
        if(bf_rate_i_er[j] > bf_rate_i_er_lim[j]) then
            bf_rate_i_er[j] = bf_rate_i_er_lim[j]
        end

        if (bf_rate_i_er[j] < -bf_rate_i_er_lim[j]) then
            bf_rate_i_er[j] = -bf_rate_i_er_lim[j]
        end
        bf_rate_d_er[j] = bf_rate_p_er[j] - bf_rate_p_er_pre[j]
    end

    for j = 1,3 do
        bf_rate_pid_out[j] = bf_rate_p_er[j] * bf_rate_gain_p[j] + bf_rate_i_er[j] * bf_rate_gain_i[j] + bf_rate_d_er[j] * bf_rate_gain_d[j]
    end
    for j = 1,3 do
        bf_rate_p_er_pre[j] = bf_rate_p_er[j]
    end

    return {bf_rate_pid_out[1], bf_rate_pid_out[2], bf_rate_pid_out[3]}
end

function get_bf_quaternion_attitude_p_controller(e_target)
    e_body[3] = ahrs:get_yaw()
    e_body[1] = ahrs:get_roll()
    e_body[2] = ahrs:get_pitch()

    q_body = get_e2q_conv(e_body)
    q_target = get_e2q_conv(e_target)

    q_body_con[1] =  -q_body[1]
    q_body_con[2] =  -q_body[2]
    q_body_con[3] =  -q_body[3]
    q_body_con[4] =  q_body[4]
    temp = get_quat_product(q_body_con, q_target)

    q_ctr_out = temp
    attitude_p_gain = 12

    --gcs:send_text(0, string.format("[L]now target    r:%.1f, p:%.1f, y:%.1f", e_target[1], e_target[2], e_target[3]))
    --gcs:send_text(0, string.format("[L]now body      r:%.1f, p:%.1f, y:%.1f", e_body[1], e_body[2], e_body[3]))
    --gcs:send_text(0, string.format("[L]now q_target q1:%.3f, q2:%.3f, q3:%.3f, q4:%.3f", q_target[1], q_target[2], q_target[3],q_target[4]))
    --gcs:send_text(0, string.format("[L]now q_body   q1:%.3f, q2:%.3f, q3:%.3f, q4:%.3f", q_body[1], q_body[2], q_body[3],q_body[4]))
    --gcs:send_text(0, string.format("[L]now temp     q1:%.3f, q2:%.3f, q3:%.3f, q4:%.3f", temp[1], temp[2], temp[3],temp[4]))
    --gcs:send_text(0, string.format("[L]now q_out    q1:%.3f, q2:%.3f, q3:%.3f, q4:%.3f", q_ctr_out[1], q_ctr_out[2], q_ctr_out[3],q_ctr_out[4]))



    return {math.deg(q_ctr_out[1]*attitude_p_gain), math.deg(q_ctr_out[2]*attitude_p_gain), math.deg(q_ctr_out[3]*attitude_p_gain)}
end

function mixer_quad_x(x_rate,y_rate,z_rate,throttle_pwm)
    pwm_1_out = -x_rate*roll_scale  + y_rate*pitch_scale + z_rate*yaw_scale + throttle_pwm
    pwm_2_out =  x_rate*roll_scale  - y_rate*pitch_scale + z_rate*yaw_scale + throttle_pwm
    pwm_3_out =  x_rate*roll_scale  + y_rate*pitch_scale - z_rate*yaw_scale + throttle_pwm
    pwm_4_out = -x_rate*roll_scale  - y_rate*pitch_scale - z_rate*yaw_scale + throttle_pwm

    pwm_1_out = math.floor(pwm_1_out)
    pwm_2_out = math.floor(pwm_2_out)
    pwm_3_out = math.floor(pwm_3_out)
    pwm_4_out = math.floor(pwm_4_out)

    if (pwm_1_out > pwm_max) then
        pwm_1_out = pwm_max
    elseif (pwm_1_out < pwm_min) then
        pwm_1_out = pwm_min
    end
    if (pwm_2_out > pwm_max) then
        pwm_2_out = pwm_max
    elseif (pwm_2_out < pwm_min) then
        pwm_2_out = pwm_min
    end
    if (pwm_3_out > pwm_max) then
        pwm_3_out = pwm_max
    elseif (pwm_3_out < pwm_min) then
        pwm_3_out = pwm_min
    end
    if (pwm_4_out > pwm_max) then
        pwm_4_out = pwm_max
    elseif (pwm_4_out < pwm_min) then
        pwm_4_out = pwm_min
    end
    --gcs:send_text(0, string.format("[L]now pwm1:%.1f, pwm2:%.1f, pwm3:%.1f, pwm4:%.1f", pwm_1_out, pwm_2_out, pwm_3_out,pwm_4_out))

    return {pwm_1_out, pwm_2_out, pwm_3_out, pwm_4_out}
end

function mixer_quad_plus(x_rate,y_rate,z_rate,throttle_pwm)
    pwm_1_out = -x_rate*roll_scale + z_rate*yaw_scale + throttle_pwm
    pwm_2_out =  x_rate*roll_scale + z_rate*yaw_scale + throttle_pwm
    pwm_3_out =  y_rate*pitch_scale  - z_rate*yaw_scale + throttle_pwm
    pwm_4_out = -y_rate*pitch_scale  - z_rate*yaw_scale + throttle_pwm

    pwm_1_out = math.floor(pwm_1_out)
    pwm_2_out = math.floor(pwm_2_out)
    pwm_3_out = math.floor(pwm_3_out)
    pwm_4_out = math.floor(pwm_4_out)

    if (pwm_1_out > pwm_max) then
        pwm_1_out = pwm_max
    elseif (pwm_1_out < pwm_min) then
        pwm_1_out = pwm_min
    end
    if (pwm_2_out > pwm_max) then
        pwm_2_out = pwm_max
    elseif (pwm_2_out < pwm_min) then
        pwm_2_out = pwm_min
    end
    if (pwm_3_out > pwm_max) then
        pwm_3_out = pwm_max
    elseif (pwm_3_out < pwm_min) then
        pwm_3_out = pwm_min
    end
    if (pwm_4_out > pwm_max) then
        pwm_4_out = pwm_max
    elseif (pwm_4_out < pwm_min) then
        pwm_4_out = pwm_min
    end
    --gcs:send_text(0, string.format("[L]now pwm1:%.1f, pwm2:%.1f, pwm3:%.1f, pwm4:%.1f", pwm_1_out, pwm_2_out, pwm_3_out,pwm_4_out))

    return {pwm_1_out, pwm_2_out, pwm_3_out, pwm_4_out}
end

function get_v4d_norm(vec4d)
    x = vec4d[1]
    y = vec4d[2]
    z = vec4d[3]
    w = vec4d[4]
    norm = math.sqrt(math.pow(x,2) + math.pow(y,2) + math.pow(z,2)+ math.pow(w,2))
    return norm
end

function get_q2e_conv(q)
    e = {0,0,0}
    e[1] = (180/math.pi)*math.atan2( 2*(q[4]*q[1]) , (1-2*(math.pow(q[1],2)+math.pow(q[2],2))))
    a = 0
    a = 2*(q[4]*q[2])
    if(a>=1) then
        a = 1
    elseif(a<=-1) then
        a = -1;
    end
    e[2] = (180/math.pi)*math.asin(a);
    e[3] = 0;
    return e
end

function get_e2q_conv(e) -- e = {roll,pitch,yaw} in radian
    --ref https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    cy = math.cos(e[3] * 0.5)
    sy = math.sin(e[3] * 0.5)
    cp = math.cos(e[2] * 0.5)
    sp = math.sin(e[2] * 0.5)
    cr = math.cos(e[1] * 0.5)
    sr = math.sin(e[1] * 0.5)

    q = {0,0,0,0}
    q[4] = cr * cp * cy + sr * sp * sy
    q[1] = sr * cp * cy - cr * sp * sy
    q[2] = cr * sp * cy + sr * cp * sy
    q[3] = cr * cp * sy - sr * sp * cy
    --gcs:send_text(0, string.format("[L]now q1:%.1f, q2:%.1f, q3:%.1f, q4:%.1f", q[1], q[2], q[3], q[4]))
    return q
end

function get_v4d_normalized(vec4)
    q = {0,0,0,0}
    q[1] = vec4[1]/get_v4d_norm(vec4)
    q[2] = vec4[2]/get_v4d_norm(vec4)
    q[3] = vec4[3]/get_v4d_norm(vec4)
    q[4] = vec4[4]/get_v4d_norm(vec4)
    return q
end

function get_quat_product(ql, qr)
    q = {0,0,0,0}
    q[1] = (ql[4]*qr[1]) + (ql[1]*qr[4]) - (ql[3]*qr[2]) + (ql[2]*qr[3])
    q[2] = (ql[4]*qr[2]) + (ql[2]*qr[4]) - (ql[1]*qr[3]) + (ql[3]*qr[1])
    q[3] = (ql[4]*qr[3]) + (ql[3]*qr[4]) - (ql[2]*qr[1]) + (ql[1]*qr[2])
    q[4] = (ql[4]*qr[4]) - ((ql[1]*qr[1])+(ql[2]*qr[2])+(ql[3]*qr[3]))
    return q
end


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
                        fast_cycle_cnt = 0
                    else
                        cycle_cnt = 0
                        fast_cycle_cnt = 0
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
            fast_cycle_cnt = fast_cycle_cnt + 1
            if (fast_cycle_cnt%20 == 0) then
                cycle_cnt = cycle_cnt + 1
            end
            
        -- *** acrobat ***
        elseif (test_mode == TESTMODE_ACROBAT2) then
            -- control
            if (cycle_cnt < 10) then  -- stabilize
                vehicle:set_target_angle_and_climbrate(0, 0, 0, 0, FALSE, 0)
            elseif (cycle_cnt == 10) then  -- Roll left
                gcs:send_text(0, "[L]----------Roll Left----------")
                SRV_Channels:set_output_pwm_chan_timeout( 0, 1600, 300)    -- right    ccw
                SRV_Channels:set_output_pwm_chan_timeout( 1, 1300, 300)    -- left     ccw
                SRV_Channels:set_output_pwm_chan_timeout( 2, 1550, 300)    -- front    cw
                SRV_Channels:set_output_pwm_chan_timeout( 3, 1550, 300)    -- rear     cw
            elseif (cycle_cnt == 50) then  -- Roll right
                gcs:send_text(0, "[L]----------Roll Right----------")
                SRV_Channels:set_output_pwm_chan_timeout( 0, 1300, 300)    -- right    ccw
                SRV_Channels:set_output_pwm_chan_timeout( 1, 1600, 300)    -- left     ccw
                SRV_Channels:set_output_pwm_chan_timeout( 2, 1550, 300)    -- front    cw
                SRV_Channels:set_output_pwm_chan_timeout( 3, 1550, 300)    -- rear     cw
            elseif (cycle_cnt == 90) then
                gcs:send_text(0, "[L]----------Backflip----------")
                SRV_Channels:set_output_pwm_chan_timeout( 0, 1550, 300)    -- right    ccw
                SRV_Channels:set_output_pwm_chan_timeout( 1, 1550, 300)    -- left     ccw
                SRV_Channels:set_output_pwm_chan_timeout( 2, 1600, 300)    -- front    cw
                SRV_Channels:set_output_pwm_chan_timeout( 3, 1300, 300)    -- rear     cw
            elseif (cycle_cnt == 130) then
                gcs:send_text(0, "[L]----------Forward Rotation----------")
                SRV_Channels:set_output_pwm_chan_timeout( 0, 1550, 300)    -- right    ccw
                SRV_Channels:set_output_pwm_chan_timeout( 1, 1550, 300)    -- left     ccw
                SRV_Channels:set_output_pwm_chan_timeout( 2, 1300, 300)    -- front    cw
                SRV_Channels:set_output_pwm_chan_timeout( 3, 1600, 300)    -- rear     cw
            elseif (cycle_cnt > 170) then
                cycle_cnt = 0
                fast_cycle_cnt = 0
                gcs:send_text(0, "[L]SWOOP")
                test_mode = TESTMODE_ACROBAT3
            end
            fast_cycle_cnt = fast_cycle_cnt + 1
            if (fast_cycle_cnt%20 == 0) then
                cycle_cnt = cycle_cnt + 1
            end
            
        -- *** swoop ***
        elseif (test_mode == TESTMODE_ACROBAT3) then
            if cycle_cnt < 200 then
                e_target = {0.0,0.0,0}
            elseif cycle_cnt < 400 then
                --------# change heading
                e_target = {0.0,0.0,math.pi*0.5}
            elseif cycle_cnt < 600 then
                --------# change heading
                e_target = {0.0,0.0,0.0}
            elseif cycle_cnt < 800 then
                --------# change pitch
                e_target = {0.0,math.pi*0.5,0.0}
            elseif cycle_cnt < 900 then
                --------# change pitch
                e_target = {0.0,0.0,0.0}
            elseif cycle_cnt < 1000 then
                --------# change pitch
                e_target = {0.0,-math.pi*0.5,0.0}
            elseif cycle_cnt < 1100 then
                --------# change pitch
                e_target = {0.0,0.0,0.0}
            elseif cycle_cnt < 1200 then
                --------# change roll
                e_target = {3.14*0.6,0.0,0.0}
            elseif cycle_cnt < 1300 then
                --------# change roll
                e_target = {0.0,0.0,0.0}
            elseif cycle_cnt < 1400 then
                --------# change roll
                e_target = {-3.14*0.6,0.0,0.0}
            elseif cycle_cnt < 1500 then
                --------# change roll
                e_target = {0.0,0.0,0.0}
            elseif cycle_cnt < 1600 then
                --------# flip landing
                e_target = {math.pi*0.25,0.0,0.0}
            elseif cycle_cnt < 1700 then
                --------# flip landing
                e_target = {math.pi*0.5,0.0,0.0}
            elseif cycle_cnt < 1800 then
                --------# flip landing
                e_target = {math.pi*0.75,0.0,0.0}
            elseif cycle_cnt < 1900 then
                --------# flip landing
                e_target = {math.pi,0.0,0.0}
            else
                --------# flip landing
                e_target = {math.pi,0.0,0.0}
            end

            -- set target euler angle (r,p,y), unit: (rad)
            omega_target_deg = get_bf_quaternion_attitude_p_controller(e_target)
            -- set target angle rate(x,y,z), unit: (deg/s)
            bf_rate_pid_out = get_bf_rate_pid_controller(omega_target_deg[1],omega_target_deg[2],omega_target_deg[3])
            -- convert pid output to pwm (x,y,z,throttle)
            pwm_output = mixer_quad_plus(bf_rate_pid_out[1], bf_rate_pid_out[2], bf_rate_pid_out[3],1800)
            -- output pwm (x,y,z,throttle)
            SRV_Channels:set_output_pwm_chan_timeout( 0, math.floor(pwm_output[1]),5)
            SRV_Channels:set_output_pwm_chan_timeout( 1, math.floor(pwm_output[2]),5)
            SRV_Channels:set_output_pwm_chan_timeout( 2, math.floor(pwm_output[3]),5)
            SRV_Channels:set_output_pwm_chan_timeout( 3, math.floor(pwm_output[4]),5)

            -- flip landing
            pos = ahrs:get_position()
            home = ahrs:get_home()
            alt = pos:alt() - home:alt()

            --gcs:send_text(0, string.format("[L]now alt :%.1f", alt))
            --gcs:send_text(0, string.format("[L]now omega_out x:%.1f, y:%.1f, z:%.1f", omega_target_deg[1], omega_target_deg[2], omega_target_deg[3]))
            if  alt < 4000 then
                cycle_cnt = 0
                test_mode = TESTMODE_RTL
            end

            cycle_cnt = cycle_cnt +1

        -- *** RTL ***
        elseif (test_mode == TESTMODE_RTL) then
            vehicle:set_mode(9)
            test_mode = TESTMODE_FINISH
            gcs:send_text(0, "[L]LAND")
        end
    end

    return update, 5
end

return update()
