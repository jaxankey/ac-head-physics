local scriptSettings = ac.INIConfig.scriptSettings()

local globalSettings = scriptSettings:mapSection('GLOBAL ADJUSTMENTS', {
  GLOBAL_FREQUENCY_SCALE = 1,
  GLOBAL_DAMPING_SCALE   = 1.0,
  GLOBAL_BANDWIDTH_SCALE = 0.0,
})

local pitchSettings = scriptSettings:mapSection('PITCH PHYSICS', {
  PITCH_ENABLED   = 1,
  PITCH_FREQUENCY = 0.7,
  PITCH_DAMPING   = 1.0,
  PITCH_LIMIT     = 30, -- degrees
  PITCH_TRACKING  = 1,
  PITCH_BANDWIDTH = 20,
})
local rollSettings = scriptSettings:mapSection('ROLL PHYSICS', {
  ROLL_ENABLED    = 1,
  ROLL_FREQUENCY  = 1,
  ROLL_DAMPING    = 1.0,
  ROLL_LIMIT      = 30, -- degrees
  ROLL_TRACKING   = 0,
  ROLL_BANDWIDTH  = 20,
})
local yawSettings = scriptSettings:mapSection('YAW PHYSICS', {
  YAW_ENABLED     = 0,
  YAW_FREQUENCY   = 1.0, -- Filter frequency for wobbles (faster wobbles are reduced). The puke-wobbles are often ~1-2 Hz.
  YAW_DAMPING     = 1,
  YAW_LIMIT       = 30, -- degrees
  YAW_TRACKING    = 1,
  YAW_BANDWIDTH   = 20,
})

local forwardSettings = scriptSettings:mapSection('FORWARD PHYSICS', {
  FORWARD_ENABLED      = 1,
  FORWARD_FREQUENCY    = 0.8,  -- Resonance frequency (Hz); lower for more smoothing
  FORWARD_DAMPING      = 1.0,  -- Damping parameter for head movement. 1 is critically damped (exponential decay), below one oscillates
  FORWARD_LIMIT        = 0.25, -- Maximum head deviation from front-back G-forces (meters). Also adds some nonlinear spring.
  FORWARD_BANDWIDTH    = 20,
})

local horizontalSettings = scriptSettings:mapSection('HORIZONTAL PHYSICS', {
  HORIZONTAL_ENABLED   = 1,
  HORIZONTAL_FREQUENCY = 1.3,
  HORIZONTAL_DAMPING   = 1.0,
  HORIZONTAL_LIMIT     = 0.12,
  HORIZONTAL_BANDWIDTH = 20,
})

local verticalSettings = scriptSettings:mapSection('VERTICAL PHYSICS', {
  VERTICAL_ENABLED     = 1,
  VERTICAL_FREQUENCY   = 1,
  VERTICAL_DAMPING     = 1.0,
  VERTICAL_LIMIT       = 0.14,
  VERTICAL_BANDWIDTH   = 20,
})

local horizonSettings = scriptSettings:mapSection('HORIZON LOCK', {
  HORIZON_PITCH = 0,
  HORIZON_ROLL  = 0,
})

local lookAheadSettings = scriptSettings:mapSection('LOOK AHEAD', {
  DRIFT_SCALE = 0,
  DRIFT_MAX_ANGLE = 90,
  VR_ENABLE_DRIFT = 1,
  STEER_SCALE = 0,
  STEER_MAX_ANGLE = 15,
  VR_ENABLE_STEER = 1,
  TRACK_SCALE = 0,
  TRACK_DISTANCE = 10,
  TRACK_MAX_ANGLE = 30,
  VR_ENABLE_TRACK = 1,
  MAX_ANGLE   = 90,
  MAX_SPEED   = 1,
})

local effectSettings = scriptSettings:mapSection('EFFECTS', {
  GEAR_BUCK_ENABLED = 0,
  GEAR_BUCK_SCALE = 0,
  GEAR_BUCK_VERTICAL = 0,
  GEAR_BUCK_FORWARD = 0,
  GEAR_BUCK_PITCH = 0,
  GEAR_BUCK_FREQUENCY = 4,
  GEAR_BUCK_DAMPING = 0.5,
  GEAR_BUCK_GAS_SCALE = 0.5,
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
  if max    == 0   then return value end -- Disabled

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

-- Increases to the max value then returns the max
-- works in +/- directions.
function hard_clip(x, max)
  if target == nil then target = 0 end

  -- Beyond these limits return a constant  
  if x >  max then return max end
  if x < -max then return max end
  return x

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
FilterLowPass = {
  time_constant  = 2.0, -- How long until a change in value returns to zero
  previous_input = 0,   -- Previous input value
  value          = 0,   -- Previous output value
}

-- Function to create an instance
function FilterLowPass:new(time_constant)

  -- Boilerplate class creation lua nonsense
  local instance = setmetatable({}, { __index = FilterLowPass })

  -- Initialization
  instance.time_constant = time_constant

  return instance
end

-- Function that evolves the high pass 
function FilterLowPass:evolve(dt, input, step_limit)

  -- Disabled
  if self.time_constant == 0 then
    self.value = input
    self.previous_input = input
    return self.value
  end

  -- Constant for easier coding
  -- This is close to zero for long time constants, 
  -- and close to 1 for short time constants
  local a = dt/(self.time_constant+dt)

  -- Get the new output (updating previous)
  if step_limit ~= nil then self.value = self.value + soft_clip(a*(input-self.value), step_limit)
  else                      self.value = self.value +           a*(input-self.value) end
  self.previous_input = input

  return self.value
end

-- Resets the variables
function FilterLowPass:reset()
  self.value          = 0
  self.previous_input = 0
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
function FilterHighPass:evolve(dt, input, step_limit)

  -- Disabled
  if self.time_constant == 0 then
    self.value = input
    self.previous_input = input
    return self.value
  end

  -- Constant for easier coding
  local a = self.time_constant/(self.time_constant+dt)

  -- Get the new output (updating previous)
  if step_limit ~= nil then self.value = self.value + soft_clip((a-1)*self.value + a*(input-self.previous_input), step_limit)
  else                      self.value = self.value +           (a-1)*self.value + a*(input-self.previous_input) end
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
local pitch_tau_hp = 0
local roll_tau_hp  = 0
local yaw_tau_hp   = 0
if pitchSettings.PITCH_TRACKING > 0 then 
  pitch_tau_hp =    1/(2*pi*head.pitch.frequency) 
  head.pitch.damping   = head.pitch.damping*0.5   -- Optimization stabilization frequency dependence
  head.pitch.frequency = head.pitch.frequency*1.2 -- Makes the responses equal closer to the user-set frequency
end
if rollSettings.ROLL_TRACKING  > 0 then 
  roll_tau_hp    = 1/(2*pi*head.roll.frequency)
  head.roll.damping   = head.roll.damping*0.5
  head.roll.frequency = head.roll.frequency*1.2
end
if yawSettings.YAW_TRACKING > 0 then
  yaw_tau_hp    = 1/(2*pi*head.yaw.frequency)
  head.yaw.damping   = head.yaw.damping*0.5
  head.yaw.frequency = head.yaw.frequency*1.2
end
--
-- Dyanmical variables for calculating acceleration transients
local transient = {
  x     = FilterHighPass:new(advancedSettings.HORIZONTAL_RECENTER_TIME),
  y     = FilterHighPass:new(advancedSettings.VERTICAL_RECENTER_TIME),
  z     = FilterHighPass:new(advancedSettings.FORWARD_RECENTER_TIME),
  pitch = FilterHighPass:new(pitch_tau_hp),
  roll  = FilterHighPass:new(roll_tau_hp),
  yaw   = FilterHighPass:new(yaw_tau_hp),
}


-- Needed for bandwidth limits
local pitch_bw   = pitchSettings.PITCH_BANDWIDTH*globalSettings.GLOBAL_BANDWIDTH_SCALE
local roll_bw    = rollSettings.ROLL_BANDWIDTH  *globalSettings.GLOBAL_BANDWIDTH_SCALE
local yaw_bw     = yawSettings.YAW_BANDWIDTH    *globalSettings.GLOBAL_BANDWIDTH_SCALE
local forward_bw    = forwardSettings.FORWARD_BANDWIDTH      *globalSettings.GLOBAL_BANDWIDTH_SCALE
local horizontal_bw = horizontalSettings.HORIZONTAL_BANDWIDTH*globalSettings.GLOBAL_BANDWIDTH_SCALE
local vertical_bw   = verticalSettings.VERTICAL_BANDWIDTH    *globalSettings.GLOBAL_BANDWIDTH_SCALE
--
local pitch_tau_lp = 0
local roll_tau_lp  = 0
local yaw_tau_lp   = 0
local forward_tau_lp    = 0
local horizontal_tau_lp = 0
local vertical_tau_lp   = 0
if pitch_bw > 0 then pitch_tau_lp = 1/(2*pi*pitch_bw) end
if roll_bw  > 0 then roll_tau_lp  = 1/(2*pi*roll_bw)  end
if yaw_bw   > 0 then yaw_tau_lp   = 1/(2*pi*yaw_bw)   end
if forward_bw    > 0 then forward_tau_lp    = 1/(2*pi*forward_bw)    end
if horizontal_bw > 0 then horizontal_tau_lp = 1/(2*pi*horizontal_bw) end
if vertical_bw   > 0 then vertical_tau_lp   = 1/(2*pi*vertical_bw)   end
--
-- Low-pass filters for bandwidth limitations
local lowpass = {
  x     = FilterLowPass:new(horizontal_tau_lp),
  y     = FilterLowPass:new(vertical_tau_lp),
  z     = FilterLowPass:new(forward_tau_lp),
  pitch = FilterLowPass:new(pitch_tau_lp),
  roll  = FilterLowPass:new(roll_tau_lp),
  yaw   = FilterLowPass:new(yaw_tau_lp),
}

-- Precomputed scale factors (doesn't save much time but oh well looks cleaner)
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
local last_yaw_lookahead = 0
local last_car_position = nil

-- This is used to more smoothly return the look-ahead angle to center at low speed 
local lookahead_drift_track = FilterSpring:new(1.0, 1.0, 0.0)
--local last_yaw_drift_track = 0 -- used for exponential decay

-- EFFECTS
local last_gear = 0
local gear_bucker = FilterSpring:new(effectSettings.GEAR_BUCK_FREQUENCY, effectSettings.GEAR_BUCK_DAMPING, 0)



--------------------------------------------------------------------------------------------------------------------------------------
function script.update(dt, mode, turnMix)

  -- In replays, dt can be zero when stopping etc.
  -- ac.debug('dt intial', dt)
  if dt == 0 then return end

  local vr_enabled = ac.getSim().isVRConnected
  --ac.debug('VR', {ac.getSim().isVRMode, ac.getSim().isVRConnected, vr_enabled})

  -- Stuff I learned:
      -- neck.look, neck.up, neck.side, car.look, car.up, car.side are orthogonal unit vectors
         -- for the orientation of the car and neck. They are matched to each other at the start of this 
         -- update function.
      -- dt can sometimes be zero in this function, so be careful!
      -- We use vec3:addScaled to avoid creating new vectors, in case CSP maintains a handle on them.

  -- REUSABLE VARIABLES
  local cos_angle, sin_angle, target, dpitch, droll, dyaw


  -----------------------------------------------------------------
  -- HEAD PHYSICS
  -- These angles we will eventually become neck rotations away from colinear with the car
  local pitch_physics = 0
  local roll_physics  = 0
  local yaw_physics   = 0
  local frameIndexChange = math.abs(ac.getSim().replayCurrentFrame - lastFrameIndex)

  -- If we just jumped to a new location, changed cars, or haven't been in the cockpit for awhile, reset stuff.
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

    lowpass.x:reset()
    lowpass.y:reset()
    lowpass.z:reset()
    lowpass.pitch:reset()
    lowpass.roll:reset()
    lowpass.yaw:reset()

    -- Reset the last car values so the car isn't thought to be rotating
    last_car_look = car.look:clone()
    last_car_up   = car.up  :clone()

    lookahead_drift_track:reset()
    --last_yaw_lookahead = 0

    first_run = false
    last_car_index = car.index
    last_clock = os.clock()
    lastFrameIndex = ac.getSim().replayCurrentFrame

    last_gear = car.gear
    gear_bucker:reset()
    return
  end

 



  -- DISPLACEMENT PHYSICS
  -- 
  -- For each of these, high-pass filter the car acceleration to get transients (unless
  -- the recenter time is set to zero (by default), in which case we just use acceleration),
  -- then evolve the position under this acceleration and displace the head.
  --
  -- car.acceleration (unlike rotation velocity?) seems to have its axes aligned with the car
  --  x = car.side
  --  y = car.up
  --  z = car.look
  --
  -- IMPORTANT: Using car.acceleration has the disadvantage that the physics engine doesn't
  -- coincide with the frame rate, which can cause jitters when frame rate is low. 
  -- These are not as noticeable for center-of-mass motion. 
  -- The only other option is to calculate acceleration manually with two steps of 
  -- different dt, which may introduce lag, but we could also maybe implement an 
  -- accumulation approach like for rotations.
  if horizontalSettings.HORIZONTAL_ENABLED == 1 then

    -- If enabled, use only the transients in acceleration, which will restore the head to
    -- equilibrium on constant acceleration
    transient.x:evolve(dt, car.acceleration.x, displacement_step_limit)

    -- Evolove harmonic oscillator with target 0 and applied acceleration from above
    -- nominally horizontal_scale=1, and transient is just acceleration.
    head.x:evolve(dt, 0, horizontal_scale*transient.x.value)

    -- We do a final low pass on the "floating head" so that the head 
    -- stays locked to the car for high frequencies
    lowpass.x:evolve(dt, head.x.value)

    -- Adjust head backwards by the value
    neck.position:addScaled(car.side, -lowpass.x.value)
  end
  -- Same for the other two DOF
  if verticalSettings.VERTICAL_ENABLED == 1 then
    transient.y:evolve(dt, car.acceleration.y, displacement_step_limit)
    head.y:evolve(dt, 0, vertical_scale*transient.y.value)
    lowpass.y:evolve(dt, head.y.value)
    neck.position:addScaled(car.up, -lowpass.y.value)
  end
  if forwardSettings.FORWARD_ENABLED == 1 then
    transient.z:evolve(dt, car.acceleration.z, displacement_step_limit)
    head.z:evolve(dt, 0, forward_scale*transient.z.value)
    lowpass.z:evolve(dt, head.z.value)
    neck.position:addScaled(car.look, -lowpass.z.value)
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
    -- Get how far the car has rotated
    dpitch = math.dot(math.cross(car.look, last_car_look), car.side)

    -- 1-2: Accumulate rotation angle from the car, optionally high-passing (enabled for eye-tracking)
    -- Limit: if eye-tracking is disabled, the evolve() statement just returns dpitch/dt
    head.pitch.value = head.pitch.value + transient.pitch:evolve(dt, dpitch/dt) * dt

    -- 3: Let the head try to relax back to center by harmonic motion.
    head.pitch:evolve(dt, 0, 0)
    lowpass.pitch:evolve(dt, head.pitch.value)

    -- Prep for 4: Get the base value to add to the look and side vectors
    pitch_physics = lowpass.pitch.value * (1-horizonSettings.HORIZON_PITCH) -- When horizon lock is enabled, decrease the physics in proportion
  end

  -- Roll dynamics: Evolve toward the car's side.y (vertical) value, and add the difference
  if rollSettings.ROLL_ENABLED == 1 then
    droll = math.dot(math.cross(car.up, last_car_up), car.look)
    head.roll.value = head.roll.value + transient.roll:evolve(dt, droll/dt) * dt
    head.roll:evolve(dt, 0, 0)
    lowpass.roll:evolve(dt, head.roll.value)
    roll_physics = lowpass.roll.value * (1-horizonSettings.HORIZON_ROLL) -- When horizon lock is enabled, decrease the physics proportion.
  end

  -- Yaw dynamics
  if yawSettings.YAW_ENABLED == 1 then
    dyaw = math.dot(math.cross(car.look, last_car_look), car.up)
    head.yaw.value = head.yaw.value + transient.yaw:evolve(dt, dyaw/dt) * dt
    head.yaw:evolve(dt, 0, 0)
    lowpass.yaw:evolve(dt, head.yaw.value)
    yaw_physics = lowpass.yaw.value
  end

  -- 4: Extra rotations from pivots
  if advancedSettings.FORWARD_PITCH_PIVOT   ~= 0 then pitch_physics = pitch_physics - lowpass.z.value/advancedSettings.FORWARD_PITCH_PIVOT   end
  if advancedSettings.HORIZONTAL_ROLL_PIVOT ~= 0 then roll_physics  = roll_physics  + lowpass.x.value/advancedSettings.HORIZONTAL_ROLL_PIVOT end
  if advancedSettings.HORIZONTAL_YAW_PIVOT  ~= 0 then yaw_physics   = yaw_physics   + lowpass.x.value/advancedSettings.HORIZONTAL_YAW_PIVOT  end


  ------------------------------------------------------------------
  -- HORIZON LOCK

  -- HORIZON LOCK PITCH
  local pitch_horizon   = 0
  if horizonSettings.HORIZON_PITCH ~= 0 then

    --  1. Find the unit vector intersecting the world's ground plane (normal 0,1,0), and car.side with a cross product
    --     if car.side is in the plane, this will point along car.look with no y component
    --     if car.side points up this is undefined
    --     if car.side is at some random jank tilt, this will point perpendicular to side but in the plane
    target = math.cross(vec3(0,1,0),car.side)
    -- Handle divide-by-zero if side is pointing up
    if target:lengthSquared() > 1e-20 then target:normalize()
    else target = car.look:clone() end

    --  2. Find the angle of rotation about car.side that aligns car.look with this unit vector.
    cos_angle = math.dot(car.look, target)
    sin_angle = math.dot(car.up  , target)
    pitch_horizon = horizonSettings.HORIZON_PITCH * math.atan(sin_angle, cos_angle)

  end -- HORIZON LOCK PITCH

  -- HORIZON LOCK ROLL
  local roll_horizon    = 0
  if horizonSettings.HORIZON_ROLL ~= 0 then

    -- Similar to pitch, but use car.look instead of car.side to rotate
    -- 1. Find the unit vector intersecting the world's ground plane (normal 0,1,0), and car.look with a cross product
    target = math.cross(vec3(0,1,0),car.look)
    -- Handle divide-by-zero if the look is pointing up (bad things happened)
    if target:lengthSquared() > 1e-20 then target:normalize()
    else target = car.side:clone() end

    -- 2. Find teh angle of rotation about car.look that aligns car.side with the target
    cos_angle = math.dot(car.side, target)
    sin_angle = math.dot(car.up  , target)
    roll_horizon = horizonSettings.HORIZON_ROLL * math.atan(sin_angle, cos_angle)
  end -- HORIZON LOCK ROLL


  ------------------------------------------------------------------
  -- LOOK-AHEAD
  local yaw_drift = 0
  local yaw_track = 0
  local yaw_lookahead   = 0 -- Total at the end (with steering)
  local yaw_drift_track = 0 -- Just the drift and track follow components
  if lookAheadSettings.DRIFT_SCALE ~= 0 or lookAheadSettings.TRACK_SCALE ~= 0 or lookAheadSettings.STEER_SCALE ~= 0 then

    -- Use the current car position first
    if last_car_position == nil then last_car_position = car.position:clone() end

    -- Calculate the track follow and drift angles only if we're moving fast enough and there
    -- wasn't some weird glitch.
    if car.speedMs > 1 and (car.position - last_car_position):lengthSquared() > 1e-20 then

      -- DRIFT
      if lookAheadSettings.DRIFT_SCALE ~= 0 and (not vr_enabled or (vr_enabled and lookAheadSettings.VR_ENABLE_DRIFT==1)) then

        -- Get the velocity direction unit vector
        local vhat = (car.position - last_car_position):normalize()
        sin_angle = math.dot(vhat, car.side)
        cos_angle = math.dot(vhat, car.look)

        -- get the soft-clipped angle to rotate
        yaw_drift = soft_clip(math.atan(sin_angle, cos_angle)*lookAheadSettings.DRIFT_SCALE, lookAheadSettings.DRIFT_MAX_ANGLE*pi/180)

        last_car_position = car.position:clone()

      end -- DRIFT

      -- TRACK (deciphered and edited from default script)
      if lookAheadSettings.TRACK_SCALE ~= 0 and (not vr_enabled or (vr_enabled and lookAheadSettings.VR_ENABLE_TRACK==1)) then
        -- car.splinePosition seems like a floating point number for the number of laps completed.
        -- car_spline_position seems like the world coordinates for the car's current location along a spline "line" (the AI line, maybe?)
        local car_spline_position = ac.trackProgressToWorldCoordinate(car.splinePosition)

        -- spline_car_distance seems to be the distance from the car's position to the spline point (how well is the car on the "correct" line)
        -- We might imagine using this to reduce the track following when we're far from the "correct" line, e.g., in the weeds.
        -- local spline_car_distance = car_spline_position:distance(car.position)
        -- JACK: We should also reduce the effect based on how far car.look is from the spline tangent.

        -- Get the target point the specified number of meters ahead of the car along the spline.
        -- I believe the "%1" is to drop off the number of laps, so we just have the progress along the current lap
        local spline_target_far  = ac.trackProgressToWorldCoordinate((car.splinePosition + lookAheadSettings.TRACK_DISTANCE/sim.trackLengthM) % 1)
        local spline_target_near = ac.trackProgressToWorldCoordinate((car.splinePosition + 1                               /sim.trackLengthM) % 1)

        -- subtract the car spline position from the spline targets to get look direction and approximate spline tangent
        local track_lookahead_far  = (spline_target_far  - car_spline_position):normalize()
        local track_lookahead_near = (spline_target_near - car_spline_position):normalize()

        -- The angle between these is the approximate amount we need to turn our head
        yaw_track = math.dot(car.up, math.cross(track_lookahead_near, track_lookahead_far))*lookAheadSettings.TRACK_SCALE

        -- Scale down with the orientation of the car
        local overlap = math.dot(car.look, track_lookahead_near)
        if overlap > 0 then
          yaw_track = soft_clip(yaw_track*overlap^2, lookAheadSettings.TRACK_MAX_ANGLE*pi/180)
        else
          yaw_track = 0
        end
          
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

      -- Total angle including the drift and track following
      -- The combined angle will be a weighted sum basted on relative contributions
      -- If one dominates over the other, it will do most of the work
      -- If they are aligned, this method does not double the angle
      -- local ad2 = angle_drift*angle_drift
      -- local at2 = angle_track*angle_track
      -- if at2+ad2 > 1e-20 then
      --   local wd2 = ad2/(at2+ad2)
      --   local wt2 = at2/(at2+ad2)
      --   angle = math.sqrt(wd2*ad2 + wt2*at2)
      -- end -- Otherwise angle is zero.

      -- Man, this just has the fewest confusing discontinuities of anything I tried.
      yaw_drift_track = yaw_drift + yaw_track

      -- Keep track of the velocity and value for smoother return to center
      --lookahead_drift_track.velocity = (yaw_drift_track - lookahead_drift_track.value)/dt
      --lookahead_drift_track.value = yaw_drift_track

    -- If we're going below the threshold speed, have things relax back to centered.
    else
      -- Critically damped decay
      yaw_drift_track = lookahead_drift_track:evolve(dt,0,0)

      -- Exponential decay
      --yaw_drift_track = last_yaw_drift_track * (1-dt/0.25)
    end
    --last_yaw_drift_track = yaw_drift_track -- for the decay stuff

    -- COMBINE WITH STEER (added even if stopped!) to get the total angle
    if not vr_enabled or (vr_enabled and lookAheadSettings.VR_ENABLE_STEER==1) then
      yaw_lookahead = yaw_drift_track - soft_clip(car.steer*pi/180*lookAheadSettings.STEER_SCALE, lookAheadSettings.STEER_MAX_ANGLE*pi/180)
    else
      yaw_lookahead = yaw_drift_track
    end

    -- Final global soft clip
    yaw_lookahead = soft_clip(yaw_lookahead, lookAheadSettings.MAX_ANGLE*pi/180.0)

    -- At the end of this, we have calculated the target "angle". 
    -- If we allowed it to move infinitely fast, we could do rotations now.
    -- However, we want to limit the speed the head is allowed to rotate to avoid violent shaking
    --ac.debug('test', (angle - last_lookahead_angle)/(lookAheadSettings.MAX_SPEED*dt) )
    if lookAheadSettings.MAX_SPEED > 0 then
      yaw_lookahead = last_yaw_lookahead + soft_clip(yaw_lookahead - last_yaw_lookahead, lookAheadSettings.MAX_SPEED*dt)
    end

    -- Debug steady rotation.
    -- angle = last_lookahead_angle + 0.1*dt

    -- Rotate all three axes, because the in-car app and / or horizon lock may adjust the "zero" of pitch, yaw, and roll
    -- cos_angle = math.cos(angle)
    -- sin_angle = math.sin(angle)
    -- rotate_about_axis(neck.look, car.up, cos_angle, sin_angle)
    -- rotate_about_axis(neck.side, car.up, cos_angle, sin_angle)
    -- rotate_about_axis(neck.up  , car.up, cos_angle, sin_angle)
    last_yaw_lookahead = yaw_lookahead

  end -- LOOK-AHEAD STUFF

  -------------------------------------------------------------------
  -- EFFECTS
  -- These are distributed through the physics and other stuff below
  local rpm_gas_scale = (1 + 2*car.gas*effectSettings.GEAR_BUCK_GAS_SCALE) * car.rpm/car.rpmLimiter
  if effectSettings.GEAR_BUCK_ENABLED ~= 0 then
    --Gear change
    if last_gear ~= car.gear then
      -- Add an impulse proportional to the acceleration along this direction
      gear_bucker.velocity = effectSettings.GEAR_BUCK_SCALE*0.5
    end
    gear_bucker:evolve(dt, 0, 0)
    neck.position:addScaled(car.up,                                         gear_bucker.value*effectSettings.GEAR_BUCK_VERTICAL*rpm_gas_scale)
    neck.position:addScaled(car.look, -math.dot(car.acceleration, car.look)*gear_bucker.value*effectSettings.GEAR_BUCK_FORWARD*rpm_gas_scale*3)
  end

  ------------------------------------------------------------------
  -- FINAL ROTATIONS
  -- Do all the combined rotations once to save CPU
  local pitch = pitch_physics+pitch_horizon
              - math.dot(car.acceleration, car.look) * gear_bucker.value*effectSettings.GEAR_BUCK_PITCH*2*rpm_gas_scale
  cos_angle = math.cos(pitch)
  sin_angle = math.sin(pitch)
  rotate_about_axis(neck.look, car.side, cos_angle, sin_angle)
  rotate_about_axis(neck.side, car.side, cos_angle, sin_angle)
  rotate_about_axis(neck.up  , car.side, cos_angle, sin_angle)

  cos_angle = math.cos(yaw_physics+yaw_lookahead)
  sin_angle = math.sin(yaw_physics+yaw_lookahead)
  rotate_about_axis(neck.look, car.up, cos_angle, sin_angle)
  rotate_about_axis(neck.side, car.up, cos_angle, sin_angle)
  rotate_about_axis(neck.up  , car.up, cos_angle, sin_angle)

  cos_angle = math.cos(roll_physics+roll_horizon)
  sin_angle = math.sin(roll_physics+roll_horizon)
  rotate_about_axis(neck.look, car.look, cos_angle, sin_angle)
  rotate_about_axis(neck.side, car.look, cos_angle, sin_angle)
  rotate_about_axis(neck.up  , car.look, cos_angle, sin_angle)


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
  last_gear = car.gear

end



