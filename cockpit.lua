local scriptSettings = ac.INIConfig.scriptSettings()

local globalSettings = scriptSettings:mapSection('GLOBAL ADJUSTMENTS', {
  GLOBAL_FREQUENCY_SCALE = 1,
  GLOBAL_DAMPING_SCALE   = 1.0
})

local pitchSettings = scriptSettings:mapSection('PITCH PHYSICS', {
  PITCH_ENABLED   = 1,
  PITCH_FREQUENCY = 0.7,
  PITCH_DAMPING   = 1.0,
  PITCH_LIMIT     = 30, -- degrees
  PITCH_TRACKING  = 1,
})
local rollSettings = scriptSettings:mapSection('ROLL PHYSICS', {
  ROLL_ENABLED    = 1,
  ROLL_FREQUENCY  = 1,
  ROLL_DAMPING    = 1.0,
  ROLL_LIMIT      = 30, -- degrees
  ROLL_TRACKING   = 0,
})
local yawSettings = scriptSettings:mapSection('YAW PHYSICS', {
  YAW_ENABLED     = 0,
  YAW_FREQUENCY   = 1.0, -- Filter frequency for wobbles (faster wobbles are reduced). The puke-wobbles are often ~1-2 Hz.
  YAW_DAMPING     = 1,
  YAW_LIMIT       = 30, -- degrees
  YAW_TRACKING    = 1,
})

local forwardSettings = scriptSettings:mapSection('FORWARD PHYSICS', {
  FORWARD_ENABLED      = 1,
  FORWARD_FREQUENCY    = 0.8,  -- Resonance frequency (Hz); lower for more smoothing
  FORWARD_DAMPING      = 1.0,  -- Damping parameter for head movement. 1 is critically damped (exponential decay), below one oscillates
  FORWARD_LIMIT        = 0.25, -- Maximum head deviation from front-back G-forces (meters). Also adds some nonlinear spring.
})

local horizontalSettings = scriptSettings:mapSection('HORIZONTAL PHYSICS', {
  HORIZONTAL_ENABLED   = 1,
  HORIZONTAL_FREQUENCY = 1.3,
  HORIZONTAL_DAMPING   = 1.0,
  HORIZONTAL_LIMIT     = 0.12,
})

local verticalSettings = scriptSettings:mapSection('VERTICAL PHYSICS', {
  VERTICAL_ENABLED     = 1,
  VERTICAL_FREQUENCY   = 1,
  VERTICAL_DAMPING     = 1.0,
  VERTICAL_LIMIT       = 0.14,
})

local lookAheadSettings = scriptSettings:mapSection('LOOK AHEAD', {
  DRIFT_SCALE = 0,
  STEER_SCALE = 0,
  TRACK_SCALE = 0,
  TRACK_DISTANCE = 10,
  TRACK_MAX_ANGLE = 30,
  MAX_ANGLE   = 90,
  MAX_SPEED   = 1,
})

local advancedSettings = scriptSettings:mapSection('ADVANCED PARAMETERS', {
  ADVANCED_MODE            = 0,

  FORWARD_PITCH_PIVOT      = 0, -- Distance (m) between head and effective pivot point (somewhere in your chest or waist), which links displacement to rotation. Set to 0 to disable
  HORIZONTAL_ROLL_PIVOT    = 0.25, -- Same as FORWARD_PITCH_PIVOT but head tilting
  HORIZONTAL_YAW_PIVOT     = 0,   -- Locks eyes this distance ahead of the car when shifting

  FORWARD_RECENTER_TIME    = 0,    -- Time scale for recentering after a change in acceleration. 0 to disable recentering
  HORIZONTAL_RECENTER_TIME = 0,
  VERTICAL_RECENTER_TIME   = 0,

  FORWARD_SCALE            = 1,  -- Amount of response to front-back G-forces (giggity scale)
  HORIZONTAL_SCALE         = 1,
  VERTICAL_SCALE           = 1,
  POSITION_SCALE           = 1,
  ROTATION_STEP_LIMIT      = 0.5,
  DISPLACEMENT_STEP_LIMIT  = 0.5,
})


-- Constants
local pi = 3.1415926535
local pi24 = 4*pi*pi

-- Per-frame limits
local     rotation_step_limit = advancedSettings.ROTATION_STEP_LIMIT
local displacement_step_limit = advancedSettings.DISPLACEMENT_STEP_LIMIT

-- Soft clip function that is exactly linear up to half the max value.
-- Then parabolic to the max value (at x=1.5*ymax). target is the center value
function soft_clip(value, max, target)
  if target == nil then target = 0 end

  -- The location of the input value where the parabola peaks
  local xmax = 1.5*max

  -- Recentered value around the target
  local x = value-target

  -- Beyond these limits return a constant  
  if x >  xmax then return target+max end
  if x < -xmax then return target-max end

  -- Below value = half of the max, return linear
  if x < max*0.5 and x > -max*0.5 then return value end

  -- Upper parabola smoothly limiting
  if x >= max*0.5 then
    return target + max-(x-xmax)*(x-xmax)/(2*max)
  else
    return target - max+(x+xmax)*(x+xmax)/(2*max)
  end
end

-- Rotate three vec3's about the vec3 k (UNIT VECTOR!) by a specified angle (radians)
function rotate_about_axis(v, k, cos_angle, sin_angle)

  -- Rodrigues' rotation formula for each point
  local r = v*cos_angle + math.cross(k,v)*sin_angle + k*math.dot(k,v)*(1-cos_angle)

  -- Modify vector in place
  v.x = r.x
  v.y = r.y
  v.z = r.z
end


-- Object to hold and evolve a dynamical variable with a spring and damper
FilterSpring = {
    value        = 0,     -- Current value
    velocity     = 0,     -- Current velocity
    target       = 0,     -- Current target value
    frequency    = 1.5,   -- Resonant frequency (Hz)
    damping      = 0.9,   -- Damping rate (1=critically damped)
    limit        = 0,     -- Optional limit (0 for no limit)
}
-- Function to create an instance
function FilterSpring:new(frequency, damping, limit)

    -- Boilerplate class creation lua nonsense
    local instance = setmetatable({}, { __index = FilterSpring })

    -- Initialization
    instance.frequency = math.min(frequency*globalSettings.GLOBAL_FREQUENCY_SCALE, 9.7) -- Keep the frequency below 10 Hz to avoid artifacts (still may be some at low frame rate)
    instance.damping   = math.min(damping  *globalSettings.GLOBAL_DAMPING_SCALE  , 1.0) -- Damping above 1 causes weirdness.
    if limit then instance.limit = limit end

    return instance
end

-- Returns the acceleration given displacement from equilibrium dx and velocity v
function FilterSpring:get_derivatives(x, v, a)
  return v, -4*pi*self.damping*self.frequency*v - pi24*self.frequency*self.frequency*x + a
end

function FilterSpring:evolve(dt, target, acceleration)

  -- If default values are 0
  if target       == nil then target       = 0 end
  if acceleration == nil then acceleration = 0 end

  -- Update the input
  self.target = target

  -- Get the deviation from the target
  local x = self.value - self.target
  local v = self.velocity

  -- Runge-Kutta 4th order parameters
  local k1x, k1v = self:get_derivatives(x,            v,            acceleration)
  local k2x, k2v = self:get_derivatives(x+0.5*dt*k1x, v+0.5*dt*k1v, acceleration)
  local k3x, k3v = self:get_derivatives(x+0.5*dt*k2x, v+0.5*dt*k2v, acceleration)
  local k4x, k4v = self:get_derivatives(x+    dt*k3x, v+    dt*k3v, acceleration)

  -- Update the value
  self.value    = soft_clip(self.value    + soft_clip((dt/6)*(k1x+2*k2x+2*k3x+k4x), rotation_step_limit), self.limit, self.target)
  self.velocity =           self.velocity +           (dt/6)*(k1v+2*k2v+2*k3v+k4v)

  -- May as well return it
  return self.value
end

-- Reset the values
function FilterSpring:reset()
  self.value     = 0
  self.velocity  = 0
  self.target    = 0
end





-- DSP High-pass filter class
FilterHighPass = {
  time_constant  = 2.0, -- How long until a change in value returns to zero
  previous_input = 0,   -- Previous input value
  value          = 0,   -- Previous output value
}

-- Function to create an instance
function FilterHighPass:new(time_constant)

  -- Boilerplate class creation lua nonsense
  local instance = setmetatable({}, { __index = FilterHighPass })

  -- Initialization
  instance.time_constant = time_constant

  return instance
end

-- Function that evolves the high pass 
function FilterHighPass:evolve(dt, input)

  -- Disabled
  if self.time_constant == 0 then
    self.value = input
    self.previous_input = input
    return self.value
  end

  -- Constant for easier coding
  local a = self.time_constant/(self.time_constant+dt)

  -- Get the new output (updating previous)
  self.value = a*self.value + a*(input-self.previous_input)
  self.previous_input = input

  return self.value
end

-- Resets the variables
function FilterHighPass:reset()
  self.value          = 0
  self.previous_input = 0
end



-- Dynamical variables for head position
local head = {
  x     = FilterSpring:new(horizontalSettings.HORIZONTAL_FREQUENCY, horizontalSettings.HORIZONTAL_DAMPING, horizontalSettings.HORIZONTAL_LIMIT),
  y     = FilterSpring:new(  verticalSettings.VERTICAL_FREQUENCY  ,   verticalSettings.VERTICAL_DAMPING  ,   verticalSettings.VERTICAL_LIMIT),
  z     = FilterSpring:new(   forwardSettings.FORWARD_FREQUENCY   ,    forwardSettings.FORWARD_DAMPING   ,    forwardSettings.FORWARD_LIMIT),
  pitch = FilterSpring:new(pitchSettings.PITCH_FREQUENCY     , pitchSettings.PITCH_DAMPING     , pitchSettings.PITCH_LIMIT*pi/180.0),
  roll  = FilterSpring:new( rollSettings.ROLL_FREQUENCY      ,  rollSettings.ROLL_DAMPING      ,  rollSettings.ROLL_LIMIT *pi/180.0),
  yaw   = FilterSpring:new(  yawSettings.YAW_FREQUENCY       ,   yawSettings.YAW_DAMPING       ,   yawSettings.YAW_LIMIT  *pi/180.0),
}




-- Needed for additional tracking enabled.
local pitch_tau = 0
local roll_tau  = 0
local yaw_tau   = 0
if pitchSettings.PITCH_TRACKING > 0 then 
  pitch_tau =    1/(2*pi*head.pitch.frequency) 
  head.pitch.damping   = head.pitch.damping*0.5   -- Optimization stabilization frequency dependence
  head.pitch.frequency = head.pitch.frequency*1.2 -- Makes the responses equal closer to the user-set frequency
end
if rollSettings.ROLL_TRACKING  > 0 then 
  roll_tau    = 1/(2*pi*head.roll.frequency)
  head.roll.damping   = head.roll.damping*0.5
  head.roll.frequency = head.roll.frequency*1.2
end
if yawSettings.YAW_TRACKING > 0 then
  yaw_tau    = 1/(2*pi*head.yaw.frequency)
  head.yaw.damping   = head.yaw.damping*0.5
  head.yaw.frequency = head.yaw.frequency*1.2
end

-- Dyanmical variables for calculating acceleration transients
local transient = {
  x     = FilterHighPass:new(advancedSettings.HORIZONTAL_RECENTER_TIME),
  y     = FilterHighPass:new(advancedSettings.VERTICAL_RECENTER_TIME),
  z     = FilterHighPass:new(advancedSettings.FORWARD_RECENTER_TIME),
  pitch = FilterHighPass:new(pitch_tau),
  roll  = FilterHighPass:new(roll_tau),
  yaw   = FilterHighPass:new(yaw_tau),
}

-- Precomputed scale factors (doesn't save much time but oh well)
local horizontal_scale = advancedSettings.HORIZONTAL_SCALE*advancedSettings.POSITION_SCALE
local vertical_scale   = advancedSettings.VERTICAL_SCALE  *advancedSettings.POSITION_SCALE
local forward_scale    = advancedSettings.FORWARD_SCALE   *advancedSettings.POSITION_SCALE

-- Several quantities we need to remember from the previous call of update()
local last_car_look = vec3(0,0,1)
local last_car_up   = vec3(0,1,0)
local first_run     = true
local last_car_index = -1
local last_clock = os.clock()
local lastFrameIndex = ac.getSim().replayCurrentFrame

-- Memory of previous look vectors
local last_lookahead_angle = 0
local last_car_position = nil




--------------------------------------------------------------------------------------------------------------------------------------
function script.update(dt, mode, turnMix)
  
  -- In replays, dt can be zero when stopping etc.
  -- ac.debug('dt intial', dt)
  if dt == 0 then return end

  
  -- Stuff I learned:
      -- neck.look, neck.up, neck.side, car.look, car.up, car.side are orthogonal unit vectors
         -- for the orientation of the car and neck. They are matched to each other at the start of this 
         -- update function.
      -- dt can sometimes be zero in this function, so be careful!
      -- We use vec3:addScaled to avoid creating new vectors, in case CSP maintains a handle on them.

  ------------------------------------------------------------------
  -- LOOK-AHEAD STUFF
  if lookAheadSettings.DRIFT_SCALE ~= 0 or lookAheadSettings.TRACK_SCALE ~= 0 or lookAheadSettings.STEER_SCALE ~= 0 then
    -- Use the current car position first
    if last_car_position == nil then last_car_position = car.position:clone() end

    -- The total angle we are supposed to look toward; this has contributions from all three dynamics below
    local angle = 0

    -- Only do look-ahead stuff if it's enabled we're going fast enough and there wasn't some weird hiccup
    if car.speedMs > 1 and (car.position - last_car_position):lengthSquared() > 1e-20 then

      -- DRIFT
      local angle_drift = 0
      if lookAheadSettings.DRIFT_SCALE ~= 0 then

        -- Get the velocity direction unit vector
        local vhat = (car.position - last_car_position):normalize()
        local sin_angle = math.dot(vhat, car.side)
        local cos_angle = math.dot(vhat, car.look)

        -- get the soft-clipped angle to rotate
        angle_drift = soft_clip(math.atan(sin_angle, cos_angle)*lookAheadSettings.DRIFT_SCALE, lookAheadSettings.MAX_ANGLE*pi/180)

        last_car_position = car.position:clone()

      end -- DRIFT

      -- Total angle
      angle = angle_drift

    -- If we're going below the threshold speed, have things relax back to centered.
    else
      -- Drift decay
      angle = last_lookahead_angle * (1-dt/0.25)
    end

    -- TRACK (deciphered and edited from default script)
    local track_angle = 0
    if lookAheadSettings.TRACK_SCALE ~= 0 then
      -- car.splinePosition seems like the fraction of the lap that is complete
      -- car_spline_position may be the car's location on that spline, so the head is apparently going to look ahead on the spline to avoid crazy head turns
      local car_spline_position = ac.trackProgressToWorldCoordinate(car.splinePosition)

      -- spline_car_distance seems to be the distance from the car's position to the spline point (how well is the car on the "correct" line)
      local spline_car_distance = car_spline_position:distance(car.position)

      -- Get the target point along the spline, the set distance ahead of us along the spline
      local spline_target = ac.trackProgressToWorldCoordinate((car.splinePosition + lookAheadSettings.TRACK_DISTANCE / sim.trackLengthM) % 1)

      -- subtract the car spline position from the spline target to get the ideal direction vector we should look
      local track_lookahead_direction = spline_target:sub(car_spline_position):normalize()

      -- Project the look-ahead onto the car's plane
      track_lookahead_direction = math.dot(track_lookahead_direction, car.look)*car.look + math.dot(track_lookahead_direction, car.side)*car.side

      -- Get the angle to the target
      local track_cos_angle = math.dot(track_lookahead_direction, car.look)
      local track_sin_angle = math.dot(track_lookahead_direction, car.side)
      track_angle = soft_clip(lookAheadSettings.TRACK_SCALE*math.atan(track_sin_angle, track_cos_angle), lookAheadSettings.TRACK_MAX_ANGLE*pi/180)

      -- local facingForward = math.pow(math.saturate(math.dot(lookAheadDelta, car.look)), 0.5)
      -- local blendNow = math.lerpInvSat(spline_car_distance, 15, 8) * facingForward
      -- lookAheadBlend = math.applyLag(lookAheadBlend, blendNow, 0.99, dt)
      -- lookAheadX = math.applyLag(lookAheadX, math.dot(lookAheadDelta * powDriftState * math.saturate(car.speedMs/10 - 0.1) * cfg.TRACK_FOLLOWING_MULT, car.side) * lookAheadBlend, 0.95, dt)
      -- local lookAheadYMult = math.dot(lookAheadDelta * 0.7, car.groundNormal) * lookAheadBlend
      -- lookAheadYMult = lookAheadYMult < 0 and lookAheadYMult / 2 or lookAheadYMult
      -- lookAheadY = math.applyLag(lookAheadY, lookAheadYMult, 0.98, dt)
      -- ----
      -- local trackFollowing = spline_target == INVALID_SPLINE_POINT and 0 or cfg.TRACK_FOLLOWING
      -- local mainTurn = turnHead * cfg.SLIDE_FOLLOWING + math.lerp(steerSmooth * cfg.STEERING_MULT, lookAheadX, trackFollowing)
    end -- TRACK

    -- COMBINE WITH STEER (added even if stopped!) to get the total angle and soft clip it to the limits
    angle = soft_clip(angle +track_angle - car.steer*pi/180*lookAheadSettings.STEER_SCALE, lookAheadSettings.MAX_ANGLE*pi/180)


    
    -- At the end of this, we have calculated the target "angle". 
    -- If we allowed it to move infinitely fast, we could do rotations now.
    -- However, we want to limit the speed the head is allowed to rotate to avoid violent shaking
    --ac.debug('test', (angle - last_lookahead_angle)/(lookAheadSettings.MAX_SPEED*dt) )
    if lookAheadSettings.MAX_SPEED > 0 then 
      angle = last_lookahead_angle + soft_clip(angle - last_lookahead_angle, lookAheadSettings.MAX_SPEED*dt)
    end

    -- Debug steady rotation.
    -- angle = last_lookahead_angle + 0.1*dt

    -- Rotate all three axes, because the in-car app 
    local cos_angle = math.cos(angle)
    local sin_angle = math.sin(angle)
    rotate_about_axis(neck.look, car.up, cos_angle, sin_angle)
    rotate_about_axis(neck.side, car.up, cos_angle, sin_angle)
    rotate_about_axis(neck.up  , car.up, cos_angle, sin_angle)

    -- Remember the last value
    last_lookahead_angle = angle
  end -- LOOK-AHEAD STUFF


  -----------------------------------------------------------------
  -- HEAD PHYSICS

  -- These angles we will eventually become neck rotations away from colinear with the car
  local pitch = 0
  local roll  = 0
  local yaw   = 0
  local frameIndexChange = math.abs(ac.getSim().replayCurrentFrame - lastFrameIndex)

  -- If we just jumped to a new location, changed cars, or haven't been in the cockpit for awhile,
  -- reset stuff.
  -- Note if the framerate of a replay is higher than the rendered rate, it can cause frameIndexChange = 2 or 3
  if first_run or car.justJumped or car.index ~= last_car_index or os.clock()-last_clock > 0.1 or frameIndexChange > 3 then -- or car.speedMs < 0.1 then
    head.pitch:reset()
    head.roll:reset()
    head.yaw:reset()
    head.x:reset()
    head.y:reset()
    head.z:reset()

    transient.x:reset()
    transient.y:reset()
    transient.z:reset()
    transient.pitch:reset()
    transient.roll:reset()
    transient.yaw:reset()

    -- Reset the last car values so the car isn't thought to be rotating
    last_car_look = car.look:clone()
    last_car_up   = car.up  :clone()

    first_run = false

  else


    -- DISPLACEMENT PHYSICS
    -- 
    -- For each of these, high-pass filter the car acceleration to get transients (unless
    -- the recenter time is set to zero, in which case we just use acceleration),
    -- then evolve the position under this acceleration and displace the head.
    --
    -- car.acceleration (unlike rotation velocity?) seems to have its axes aligned with the car
    --  x = car.side
    --  y = car.up
    --  z = car.look
    --
    -- IMPORTANT: Using car.acceleration has the disadvantage that the physics engine doesn't
    -- coincide with the frame rate, which can cause jitters when frame rate is low. 
    -- These are not as noticeable for center-of-mass motion, as much. 
    -- The only other option is to calculate acceleration manually with two steps of 
    -- different dt, which may introduce lag, but we could also maybe implement an 
    -- accumulation approach like for rotations.
    if horizontalSettings.HORIZONTAL_ENABLED == 1 then
      transient.x:evolve(dt, car.acceleration.x, displacement_step_limit)
      head.x:evolve(dt, 0, horizontal_scale*transient.x.value)
      neck.position:addScaled(car.side, -head.x.value)
    end
    if verticalSettings.VERTICAL_ENABLED == 1 then
      transient.y:evolve(dt, car.acceleration.y, displacement_step_limit)
      head.y:evolve(dt, 0, vertical_scale*transient.y.value)
      neck.position:addScaled(car.up, -head.y.value)
    end
    if forwardSettings.FORWARD_ENABLED == 1 then
      transient.z:evolve(dt, car.acceleration.z, displacement_step_limit)
      head.z:evolve(dt, 0, forward_scale*transient.z.value)
      neck.position:addScaled(car.look, -head.z.value)
    end

    -- ROTATION PHYSICS, ACCUMULATION APPROACH: Advantage that there is no singularity when pitch = 90 degrees
      -- Disadvantage that there is no tune between car vs horizon
      -- 1. Add the angular change to the roll value
      -- 2. If "extra tracking" is enabled, high-pass according to the optimal calculations.
      -- 3. Evolve the rotation as a harmonic oscillator.
      -- 4. Later, if pivots are enables, we add some extra angle from head displacement.
      -- Note we do the /dt process so that variable frame rate has a smoother evolution.

    -- Pitch dynamics: Evolve toward the car's look.y (vertical) value, and add the difference
    if pitchSettings.PITCH_ENABLED == 1 then
      -- 1-2: Accumulate rotation angle from the car, optionally high-passing.
      head.pitch.value = head.pitch.value + transient.pitch:evolve(dt,
        math.dot(math.cross(car.look, last_car_look), car.side)/dt) * dt

      -- 3: Let the head try to relax back to center by harmonic motion.
      head.pitch:evolve(dt, 0, 0)

      -- Prep for 4: Get the base value to add to the look and side vectors
      pitch = head.pitch.value

    end

    -- Roll dynamics: Evolve toward the car's side.y (vertical) value, and add the difference
    if rollSettings.ROLL_ENABLED == 1 then
      -- 1-2: Accumulate rotation angle from the car, optionally high-passing.
      head.roll.value = head.roll.value + transient.roll:evolve(dt,
        math.dot(math.cross(car.up, last_car_up), car.look)/dt) * dt

      -- 3: Let the head try to relax back to center by harmonic motion.
      head.roll:evolve(dt, 0, 0)

      -- Prep for 4: Get the base value to add to the look and side vectors
      roll = head.roll.value
    end

    -- Yaw dynamics
    if yawSettings.YAW_ENABLED == 1 then
      -- 1-2: Accumulate rotation angle from the car, optionally high-passing.
      head.yaw.value = head.yaw.value + transient.yaw:evolve(dt,
        math.dot(math.cross(car.look, last_car_look), car.up)/dt) * dt

      -- 3: Let the head try to relax back to center by harmonic motion.
      head.yaw:evolve(dt, 0, 0)

      -- Prep for 4: Get the base value to add to the look and side vectors
      yaw = head.yaw.value
    end

    -- 4: Extra rotations from pivots
    if advancedSettings.FORWARD_PITCH_PIVOT   ~= 0 then pitch = pitch - head.z.value/advancedSettings.FORWARD_PITCH_PIVOT   end
    if advancedSettings.HORIZONTAL_ROLL_PIVOT ~= 0 then roll  = roll  + head.x.value/advancedSettings.HORIZONTAL_ROLL_PIVOT end
    if advancedSettings.HORIZONTAL_YAW_PIVOT  ~= 0 then yaw   = yaw   + head.x.value/advancedSettings.HORIZONTAL_YAW_PIVOT  end

    -- At this point we have only displaced the neck according to g-forces, and calculated how far the neck should rotate relative
    -- to the equilibrium point, reckoned with respect to the car axes
    -- If there is some kind of look-ahead, like track, wheel, or drift following, the neck's rotation may 
    -- be wildly different from aligned to the car. However, we should be able to apply pitch, yaw, roll to all three
    -- of these axes to smooth out the bumps. This effectively means the stiffness and damping in the three directions
    -- will always be reckoned with respect to the car, not the look-ahead equilibrium axes of the head.
    -- The position is already handled.

    local c = math.cos(pitch)
    local s = math.sin(pitch)
    rotate_about_axis(neck.look, car.side, c, s)
    rotate_about_axis(neck.side, car.side, c, s)
    rotate_about_axis(neck.up  , car.side, c, s)

    c = math.cos(yaw)
    s = math.sin(yaw)
    rotate_about_axis(neck.look, car.up, c, s)
    rotate_about_axis(neck.side, car.up, c, s)
    rotate_about_axis(neck.up  , car.up, c, s)

    c = math.cos(roll)
    s = math.sin(roll)
    rotate_about_axis(neck.look, car.look, c, s)
    rotate_about_axis(neck.side, car.look, c, s)
    rotate_about_axis(neck.up  , car.look, c, s)


    -- Quick method with no look-ahead didn't save much overhead.
    -- else
    --   -- QUICK METHOD Do the rotations
    --   -- NOTE: This is an approximation valid for small rotations, and we rotate only 
    --   --       the most important two axes for each DOF.
    --   --       We could try rotating all 3 and try a (costly) normalized angle rotation
    --   --       rather than distance to improve non-orthogonality?
    --   neck.look:addScaled(math.cross(car.side,neck.look), pitch)
    --   neck.up  :addScaled(math.cross(car.side,neck.up  ), pitch)
    --   neck.up  :addScaled(math.cross(car.look,neck.up  ), roll )
    --   neck.side:addScaled(math.cross(car.look,neck.side), roll )
    --   neck.look:addScaled(math.cross(car.up  ,neck.look), yaw  )
    --   neck.side:addScaled(math.cross(car.up  ,neck.side), yaw  )
    -- end

  end -- End of "we should do neck physics"


  -- These tests showed me that even crashing an F2004 the lengths changed by at most ~2% from unity.
  -- So the small rotation approximation is pretty good
  -- ac.debug('look.length', neck.look:length())
  -- ac.debug('up.length', neck.up:length())
  -- ac.debug('side.length', neck.side:length())

  -- These tests showed that normal driving lead to <1% overlap between these basis vectors.
  -- In a crash, these things can be like 0.2, which is consistent with few percent in length errors,
  -- which are second order (overlap is first order)
  -- ac.debug('look-up overlap',   math.dot(neck.look, neck.up))
  -- ac.debug('look-side overlap', math.dot(neck.look, neck.side))
  -- ac.debug('side-up overlap',   math.dot(neck.side, neck.up))
  -- ac.debug('look length', neck.look:length())
  -- ac.debug('side length', neck.side:length())
  -- ac.debug('up length'  , neck.up  :length())
  -- ac.debug('dt', dt)
  -- ac.debug('neck look', neck.look)
  -- ac.debug('car look', car.look)

  -- Remember the last values (making copies)
  last_car_look  = car.look:clone()
  last_car_up    = car.up  :clone()
  last_car_index = car.index
  last_clock     = os.clock()
  lastFrameIndex = ac.getSim().replayCurrentFrame
  
end



