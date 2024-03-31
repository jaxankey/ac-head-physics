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

  POSITION_SCALE           = 1
})


-- Constants
local pi = 3.1415926535
local pi24 = 4*pi*pi

-- Soft clip function that is exactly linear up to half the max value.
-- Then parabolic to the max value (at x=1.5*ymax)
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
    instance.damping   = damping  *globalSettings.GLOBAL_DAMPING_SCALE
    if limit then instance.limit = limit end

    return instance
end

-- Function to evolve from the current value to the target value.
-- dt is the time step in seconds, assumed to be small.
function FilterSpring:evolve_target(dt, target)

    -- Update the input
    self.target = target

    -- Get the deviation from the target
    local dx = self.value - self.target
    local v  = self.velocity
    local f  = self.frequency
    local g  = self.damping

    -- Update the velocity from the acceleration (damping and spring)
    self.velocity = v - (4*pi*g*f*v + pi24*f*f*dx)*dt

    -- Update the position with this velocity, and soft clip it
    local start_value = self.value
    self.value = soft_clip(self.value + self.velocity*dt, self.limit, self.target)

    -- Soft clip the velocity too as we approach the boundary
    self.velocity = (self.value-start_value)/dt

    -- May as well return it
    return self.value
end

-- Function to evolve using an applied acceleration.
-- dt is the time step in seconds, assumed to be small.
function FilterSpring:evolve_acceleration(dt, a)

  -- The target in this case is always zero.

  -- Get the deviation from the target
  local v  = self.velocity
  local f  = self.frequency
  local g  = self.damping



  -- Update the velocity from the acceleration (damping and spring)
  self.velocity = v + (a - 4*pi*g*f*v - pi24*f*f*self.value)*dt

  -- Update the position with this velocity, and soft-clip it
  local start_value = self.value
  self.value = soft_clip(self.value + self.velocity*dt, self.limit, self.target)

  -- Soft clip the velocity too
  self.velocity = (self.value-start_value)/dt

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


function script.update(dt, mode, turnMix)

  -- Stuff I learned:
      -- neck.look, neck.up, neck.side, car.look, car.up, car.side are orthogonal unit vectors
         -- for the orientation of the car and neck. They are matched to each other at the start of this 
         -- update function.
      -- dt can sometimes be zero in this function, so be careful!
      -- We use vec3:addScaled to avoid creating new vectors, in case CSP maintains a handle on them.

  -- In replays, dt can be zero when stopping etc.
  -- ac.debug('dt intial', dt)
  if dt == 0 then return end

  -- Thes angles we will eventually become neck rotations away from colinear with the car
  local pitch = 0
  local roll  = 0
  local yaw   = 0

  local frameIndexChange = math.abs(ac.getSim().replayCurrentFrame - lastFrameIndex)

  -- If we just jumped to a new location, changed cars, or haven't been in the cockpit for awhile,
  -- reset stuff.
  if first_run or car.justJumped or car.index ~= last_car_index or os.clock()-last_clock > 0.1 or frameIndexChange > 1 then -- or car.speedMs < 0.1 then
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
    -- Note car.acceleration (unlike rotation velocity?) seems to have its axes aligned with the car
    --  x = car.side
    --  y = car.up
    --  z = car.look
    --
    -- Note using car.acceleration has the disadvantage that the physics engine doesn't
    -- coincide with the frame rate, which can cause jitters when frame rate is low. 
    -- These are not as noticeable for center-of-mass motion, as much. 
    -- The only other option is to calculate acceleration manually with two steps of 
    -- different dt, which may introduce lag, but we could also maybe implement an 
    -- accumulation approach like for rotations.
    if horizontalSettings.HORIZONTAL_ENABLED == 1 then
      transient.x:evolve(dt, car.acceleration.x)
      head.x:evolve_acceleration(dt, horizontal_scale*transient.x.value)
      neck.position:addScaled(car.side, -head.x.value)
    end
    if verticalSettings.VERTICAL_ENABLED == 1 then
      transient.y:evolve(dt, car.acceleration.y)
      head.y:evolve_acceleration(dt, vertical_scale*transient.y.value)
      neck.position:addScaled(car.up, -head.y.value)
    end
    if forwardSettings.FORWARD_ENABLED == 1 then
      transient.z:evolve(dt, car.acceleration.z)
      head.z:evolve_acceleration(dt, forward_scale*transient.z.value)
      neck.position:addScaled(car.look, -head.z.value)
    end

    -- ACCUMULATION APPROACH: Advantage that there is no singularity when pitch = 90 degrees
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
      head.pitch:evolve_target(dt, 0)

      -- Prep for 4: Get the base value to add to the look and side vectors
      pitch = head.pitch.value

    end

    -- Roll dynamics: Evolve toward the car's side.y (vertical) value, and add the difference
    if rollSettings.ROLL_ENABLED == 1 then
      -- 1-2: Accumulate rotation angle from the car, optionally high-passing.
      head.roll.value = head.roll.value + transient.roll:evolve(dt,
        math.dot(math.cross(car.up, last_car_up), car.look)/dt) * dt

      -- 3: Let the head try to relax back to center by harmonic motion.
      head.roll:evolve_target(dt, 0)

      -- Prep for 4: Get the base value to add to the look and side vectors
      roll = head.roll.value
    end

    -- Yaw dynamics
    if yawSettings.YAW_ENABLED == 1 then
      -- 1-2: Accumulate rotation angle from the car, optionally high-passing.
      head.yaw.value = head.yaw.value + transient.yaw:evolve(dt,
        math.dot(math.cross(car.look, last_car_look), car.up)/dt) * dt

      -- 3: Let the head try to relax back to center by harmonic motion.
      head.yaw:evolve_target(dt, 0)

      -- Prep for 4: Get the base value to add to the look and side vectors
      yaw = head.yaw.value
    end

    -- 4: Extra rotations from pivots
    if advancedSettings.FORWARD_PITCH_PIVOT   ~= 0 then pitch = pitch - head.z.value/advancedSettings.FORWARD_PITCH_PIVOT   end
    if advancedSettings.HORIZONTAL_ROLL_PIVOT ~= 0 then roll  = roll  + head.x.value/advancedSettings.HORIZONTAL_ROLL_PIVOT end
    if advancedSettings.HORIZONTAL_YAW_PIVOT  ~= 0 then yaw   = yaw   + head.x.value/advancedSettings.HORIZONTAL_YAW_PIVOT  end

  end

  -- Remember the last values (making copies)
  last_car_look  = car.look:clone()
  last_car_up    = car.up  :clone()
  last_car_index = car.index
  last_clock     = os.clock()
  lastFrameIndex = ac.getSim().replayCurrentFrame

  -- Do the rotations
  -- NOTE: This is an approximation, rotating the most important 2 vectors.
  --       We could try rotating all 3 and use a (costly) normalized angle rotation
  --       rather than distance to improve non-orthogonality?
  neck.look:addScaled(math.cross(car.side,neck.look), pitch)
  neck.up  :addScaled(math.cross(car.side,neck.up  ), pitch)
  neck.up  :addScaled(math.cross(car.look,neck.up  ), roll )
  neck.side:addScaled(math.cross(car.look,neck.side), roll )
  neck.look:addScaled(math.cross(car.up  ,neck.look), yaw  )
  neck.side:addScaled(math.cross(car.up  ,neck.side), yaw  )

  -- These tests showed me that even crashing an F2004 the lengths changed by at most ~2% from unity.
  -- So the small rotation approximation is pretty good
  -- ac.debug('look.length', neck.look:length())
  -- ac.debug('up.length', neck.up:length())
  -- ac.debug('side.length', neck.side:length())

  -- These tests showed that normal driving lead to <1% overlap between these basis vectors.
  -- In a crash, these things can be like 0.2, which is consistent with few percent in length errors,
  -- which are second order (overlap is first order)
  -- ac.debug('look-up overlap',   dot(neck.look, neck.up))
  -- ac.debug('look-side overlap', dot(neck.look, neck.side))
  -- ac.debug('side-up overlap',   dot(neck.side, neck.up))
  -- ac.debug('dt', dt)
  -- ac.debug('neck look', neck.look)
  -- ac.debug('car look', car.look)
end



