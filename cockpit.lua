local scriptSettings = ac.INIConfig.scriptSettings()
local settings = scriptSettings:mapSection('SETTINGS', {

  GLOBAL_FREQUENCY_SCALE = 1,
  GLOBAL_DAMPING_SCALE   = 1,

  -- Set to zero to disable, 1 to enable
  HORIZONTAL_ENABLED = 1,
  VERTICAL_ENABLED   = 1,
  FORWARD_ENABLED    = 1,
  PITCH_ENABLED      = 1,
  ROLL_ENABLED       = 1,
  YAW_ENABLED        = 1,

  -- Overall scale on position effects. 
  POSITION_SCALE = 1,

  -- Front-back motion (Giggity axis)
  FORWARD_SCALE  = 1,          -- Amount of response to front-back G-forces (giggity scale)
  FORWARD_LIMIT  = 0.2,        -- Maximum head deviation from front-back G-forces (meters). Also adds some nonlinear spring.
  FORWARD_FREQUENCY = 1.0,     -- Resonance frequency (Hz); lower for more smoothing
  FORWARD_DAMPING   = 1,       -- Damping parameter for head movement. 1 is critically damped (exponential decay), below one oscillates
  FORWARD_RECENTER_TIME = 2,   -- Time scale for recentering after a change in acceleration. 0 to disable recentering
  FORWARD_PITCH_PIVOT = 0.25,  -- Distance (m) between head and effective pivot point (somewhere in your chest or waist), which links displacement to rotation. Set to 0 to disable

  -- Left-right motion (Roxbury axis)
  HORIZONTAL_SCALE  = 1,
  HORIZONTAL_LIMIT  = 0.05,
  HORIZONTAL_FREQUENCY = 1.0,
  HORIZONTAL_DAMPING   = 1,
  HORIZONTAL_RECENTER_TIME = 2,
  HORIZONTAL_ROLL_PIVOT = 0.25, -- Same as FORWARD_PITCH_PIVOT but head tilting
  HORIZONTAL_YAW_PIVOT  = 10,   -- Locks eyes this distance ahead of the car when shifting

  -- Vertical motion (Bobblehead axis)
  VERTICAL_SCALE = 1,
  VERTICAL_LIMIT = 0.04,
  VERTICAL_FREQUENCY = 1.0,
  VERTICAL_DAMPING = 1,
  VERTICAL_RECENTER_TIME = 2,

  -- Head tilting forward
  PITCH_TRACKING  = 1, -- Additional high-pass on car rotation (0 or 1)
  PITCH_FREQUENCY = 3,
  PITCH_DAMPING   = 1,
  PITCH_LIMIT     = 30,   -- degrees

  -- Head rolling with car
  ROLL_TRACKING    = 1,
  ROLL_FREQUENCY = 3,
  ROLL_DAMPING   = 1,
  ROLL_LIMIT     = 40,      -- degrees

  -- Let's the car turn under your head to kill yaw wobbles (yawbbles). Takes some getting used to!
  YAW_TRACKING  = 1,
  YAW_FREQUENCY = 0.5,    -- Filter frequency for wobbles (faster wobbles are reduced). The puke-wobbles are often ~1-2 Hz.
  YAW_DAMPING   = 1,
  YAW_LIMIT     = 10,     -- degrees
})



-- Constants
local pi = 3.1415926535
local pi24 = 4*pi*pi

-- Soft clipping function to place bounds on any value
--  value is the input value, and the return value will
--  be bounded between +/-max
function soft_clip_old(value, max)
  -- return value*max/(1+math.abs(value))
  if value >  2*max then return  max end
  if value < -2*max then return -max end
  return value - value*math.abs(value)/(4*max)
end

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
    instance.frequency = frequency*settings.GLOBAL_FREQUENCY_SCALE
    instance.damping   = damping  *settings.GLOBAL_DAMPING_SCALE
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
  x     = FilterSpring:new(settings.HORIZONTAL_FREQUENCY, settings.HORIZONTAL_DAMPING, settings.HORIZONTAL_LIMIT),
  y     = FilterSpring:new(settings.VERTICAL_FREQUENCY  , settings.VERTICAL_DAMPING  , settings.VERTICAL_LIMIT),
  z     = FilterSpring:new(settings.FORWARD_FREQUENCY   , settings.FORWARD_DAMPING   , settings.FORWARD_LIMIT),
  pitch = FilterSpring:new(settings.PITCH_FREQUENCY     , settings.PITCH_DAMPING     , settings.PITCH_LIMIT*pi/180.0),
  roll  = FilterSpring:new(settings.ROLL_FREQUENCY      , settings.ROLL_DAMPING      , settings.ROLL_LIMIT *pi/180.0),
  yaw   = FilterSpring:new(settings.YAW_FREQUENCY       , settings.YAW_DAMPING       , settings.YAW_LIMIT  *pi/180.0),
}

-- Whether we're enabling the additional tracking high-pass
local pitch_tau
local roll_tau
local yaw_tau
if settings.PITCH_TRACKING > 0 then 
  pitch_tau = 1/(2*pi*settings.PITCH_FREQUENCY) 
  head.pitch.damping   = head.pitch.damping*0.5   -- Optimization stabilization frequency dependence
  head.pitch.frequency = head.pitch.frequency*1.2 -- Makes the responses equal closer to the user-set frequency
else pitch_tau = 0 end

if settings.ROLL_TRACKING  > 0 then 
  roll_tau  = 1/(2*pi*settings.ROLL_FREQUENCY)
  head.roll.damping   = head.roll.damping*0.5
  head.roll.frequency = head.roll.frequency*1.2
else roll_tau = 0 end

if settings.YAW_TRACKING > 0 then
  yaw_tau = 1/(2*pi*settings.YAW_FREQUENCY)
  head.yaw.damping   = head.yaw.damping*0.5
  head.yaw.frequency = head.yaw.frequency*1.2
else yaw_tau = 0 end

-- Dyanmical variables for calculating accelration transients
local transient = {
  x     = FilterHighPass:new(settings.HORIZONTAL_RECENTER_TIME),
  y     = FilterHighPass:new(settings.VERTICAL_RECENTER_TIME),
  z     = FilterHighPass:new(settings.FORWARD_RECENTER_TIME),
  pitch = FilterHighPass:new(pitch_tau),
  roll  = FilterHighPass:new(roll_tau),
  yaw   = FilterHighPass:new(yaw_tau),
}

-- Precomputed scale factors (doesn't save much time but oh well)
local horizontal_scale = settings.HORIZONTAL_SCALE*settings.POSITION_SCALE
local vertical_scale   = settings.VERTICAL_SCALE  *settings.POSITION_SCALE
local forward_scale    = settings.FORWARD_SCALE   *settings.POSITION_SCALE

-- For parsing lines like SETTING=THING
-- function split_and_strip(input, delimiter)
--   local result = {}

--   -- Loop over the tokens using this archaic bullshit string parsing.
--   for token in input:gmatch("([^" .. delimiter .. "]+)") do
--       -- Strip leading and trailing whitespaces from each token
--       local strippedToken = token:match("^%s*(.-)%s*$")
--       table.insert(result, strippedToken)
--   end

--   return result
-- end

-- Save some of this in case we want to save our own settings later.
-- local path_view = os.getenv('HOMEPATH') .. '\\Documents\\Assetto Corsa\\cfg\\cars\\' .. ac.getCarID() .. '\\view.ini'
-- local last_check = os.clock()
-- function get_eye_pitch()

--   -- Used by line loop
--   local result

--   -- Open the file in read mode
--   local file = io.open(path_view, "r")

--   -- Check if the file was opened successfully
--   if file then

--       -- Iterate over each line in the file
--       for line in file:lines() do

--         -- if the line is interesting, look for the pitch angle
--         if string.len(line) > 0 then

--             -- Get the key-value pair
--             result = split_and_strip(line, '=')

--             -- Check for the key we care about.
--             if string.match(result[1], 'ON_BOARD_PITCH_ANGLE') then 
--               file:close()
--               return tonumber(result[2])
--             end
--           end
--       end

--       -- Close the file when done
--       file:close()
--       return true
--   else
--       return false
--   end

-- end

-- Jack: This script will wig out when there are frame drops, and may produce
--       different effects for people with different frame rates. It doesn't use dt.
local last_car_look = vec3(0,0,1)
local last_car_up   = vec3(0,1,0)
local first_run     = true
local last_car_index = -1
local last_clock = os.clock()
function script.update(dt, mode, turnMix)
  
  -- The angles we will eventually add to the neck vectors, relative to the car's 
  -- side, look, and up vectors, respectively
  local pitch = 0
  local roll  = 0
  local yaw   = 0

  -- If we just jumped to a new location, changed cars, or haven't been in the cockpit for awhile,
  -- reset stuff.
  if first_run or car.justJumped or car.index ~= last_car_index or os.clock()-last_clock > 0.1 then -- or car.speedMs < 0.1 then
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
    --ac.debug('reset', true)
  else
    --ac.debug('reset', false)
    -- Displacement physics
    -- 
    -- For each of these, high-pass filter the car acceleration to get transients,
    -- then evolve the position under this acceleration and displace the head.
    --
    -- Note car.acceleration (unlike rotation velocity?) seems to have its axes aligned with the car
    --  x = car.side
    --  y = car.up
    --  z = car.look
    -- 
    -- JACK: Relying on the ac physics engine here is a bit spotty, since it doesn't 
    --       coincide with the frame rate. Can cause jitters. These are not as noticeable for
    --       center of mass motion, though. The only other option is to calculate acceleration 
    --       manually with two steps of different dt.
    if settings.HORIZONTAL_ENABLED == 1 then
      transient.x:evolve(dt, car.acceleration.x)
      head.x:evolve_acceleration(dt, horizontal_scale*transient.x.value)
      --neck.position = neck.position - head.x.value*car.side
      neck.position:addScaled(car.side, -head.x.value)
    end
    if settings.VERTICAL_ENABLED == 1 then
      transient.y:evolve(dt, car.acceleration.y)
      head.y:evolve_acceleration(dt, vertical_scale*transient.y.value)
      --neck.position = neck.position - head.y.value*car.up
      neck.position:addScaled(car.up, -head.y.value)
    end
    if settings.FORWARD_ENABLED == 1 then
      transient.z:evolve(dt, car.acceleration.z)
      head.z:evolve_acceleration(dt, forward_scale*transient.z.value)
      --neck.position = neck.position - head.z.value*car.look
      neck.position:addScaled(car.look, -head.z.value)
    end

    -- neck.look, neck.up, neck.side, car.look, car.up, car.side are orthogonal unit vectors
    -- for the orientation of the car and neck (matched to each other at the start of this 
    -- update function).

    -- Pitch dynamics: Evolve toward the car's look.y (vertical) value, and add the difference
    if settings.PITCH_ENABLED == 1 then
      -- ACCUMULATION APPROACH: Advantage that there is no singularity when roll = 90 degrees
      --                        Disadvantage that there is no tune between car vs horizon
      -- 1. Add the angular change to the pitch value, but high-pass (if enabled)
      --    it so that we are sensitive only to transients. Note we go through 
      --    the /dt process so that variable frame rate has a smoother evolution.
      -- Note that with the transient disabled, this just accumulates the total angle
      head.pitch.value = head.pitch.value + transient.pitch:evolve(dt,
        math.dot(math.cross(car.look, last_car_look), car.side)/dt) * dt

      -- 2. Let the head try to relax back to center by harmonic motion.
      head.pitch:evolve_target(dt, 0)

      -- 3. Get the base value to add to the look and side vectors
      pitch = head.pitch.value

      -- HORIZON APPROACH
      -- JACK: Get the vector that is the up vector with no pitch, i.e. perpendicular to car.side
      -- This involves rotating about car.side until the look vector is in the horizon plane.
      -- Then take an overlap between look and this vector to get the pitch. Note there 
      -- will not always be a rotation that achieves this, e.g., when the car is at 90 degrees.
      --  COULD use car.localVelocity.z or car.groundNormal
      --  Cross of the side and horizon normal to get the in-plane vectoer perpendicular to both
      --  Then cross the result with the look vector to get the pitch about the side vector
      --pitch = car.look.y - head.pitch:evolve_target(dt,car.look.y)*settings.PITCH_SCALE
    end

    -- Roll dynamics: Evolve toward the car's side.y (vertical) value, and add the difference
    if settings.ROLL_ENABLED == 1 then
      -- ACCUMULATION APPROACH: Advantage that there is no singularity when pitch = 90 degrees
      --                        Disadvantage that there is no tune between car vs horizon
      -- 1. Add the angular change to the roll value, but high-pass (if enabled)
      --    it so that we are sensitive only to transients. Note we go through 
      --    the /dt process so that variable frame rate has a smoother evolution.
      -- Note that with the transient disabled, this just accumulates the total angle
      head.roll.value = head.roll.value + transient.roll:evolve(dt,
        math.dot(math.cross(car.up, last_car_up), car.look)/dt) * dt

      -- 2. Let the head try to relax back to center by harmonic motion.
      head.roll:evolve_target(dt, 0)

      -- 3. Get the base value to add to the look and side vectors
      roll = head.roll.value

      -- HORIZON APPROACH
      --roll = -car.side.y + head.roll:evolve_target(dt,car.side.y)*settings.ROLL_SCALE
    end

    -- Yaw dynamics
    if settings.YAW_ENABLED == 1 then
      -- 1. Add the angular change to the yaw value, but high-pass 
      --    it so that we are sensitive only to transients. Note we go through 
      --    the /dt process so that variable frame rate has a smoother evolution.
      -- Note that with the transient disabled, this just accumulates the total angle
      --ac.debug('test', vec2(car.look.x-last_car_look.x, car.look.z-last_car_look.z):length())
      head.yaw.value = head.yaw.value + transient.yaw:evolve(dt,
        math.dot(math.cross(car.look, last_car_look), car.up)/dt) * dt
        --cross2d(car.look.x, car.look.z, last_car_look.x, last_car_look.z)/dt) * dt
      -- 2. Let the head try to relax back to center by harmonic motion.
      head.yaw:evolve_target(dt, 0)

      -- 3. Get the base value to add to the look and side vectors
      yaw = head.yaw.value
    end

    -- Extra rotations from pivots
    if settings.FORWARD_PITCH_PIVOT   ~= 0 then pitch = pitch - head.z.value/settings.FORWARD_PITCH_PIVOT   end
    if settings.HORIZONTAL_ROLL_PIVOT ~= 0 then roll  = roll  + head.x.value/settings.HORIZONTAL_ROLL_PIVOT end
    if settings.HORIZONTAL_YAW_PIVOT  ~= 0 then yaw   = yaw   + head.x.value/settings.HORIZONTAL_YAW_PIVOT  end

  end

  -- Remember the last values
  last_car_look  = car.look:clone()
  last_car_up    = car.up:clone()
  last_car_index = car.index
  last_clock     = os.clock()
  --last_isCameraOnboard = car.isCameraOnBoard -- Function not called when not onboard.


  -- Do the rotations
  -- JACK: Do the rotations for all 3 and rotate by the angle
  --       rather than distance to handle non-orthogonality?
  -- neck.look = small_rotation(neck.look, car.side, pitch)
  -- neck.up   = small_rotation(neck.up  , car.side, pitch)
  -- neck.up   = small_rotation(neck.up  , car.look, roll )
  -- neck.side = small_rotation(neck.side, car.look, roll )
  -- neck.look = small_rotation(neck.look, car.up  , yaw  )
  -- neck.side = small_rotation(neck.side, car.up  , yaw  )
  neck.look:addScaled(math.cross(car.side,neck.look), pitch)
  neck.up  :addScaled(math.cross(car.side,neck.up  ), pitch)
  neck.up  :addScaled(math.cross(car.look,neck.up  ), roll )
  neck.side:addScaled(math.cross(car.look,neck.side), roll )
  neck.look:addScaled(math.cross(car.up  ,neck.look), yaw  )
  neck.side:addScaled(math.cross(car.up  ,neck.side), yaw  )

  -- These tests showed me that even crashing an F2004 the lengths changed by at most ~2% from unity.
  -- So the small rotation approximation is good
  -- ac.debug('look.length', neck.look:length())
  -- ac.debug('up.length', neck.up:length())
  -- ac.debug('side.length', neck.side:length())

  -- These tests showed that normal driving lead to <1% overlap between these basis vectors.
  -- In a crash, these things can be like 0.2, which is consistent with few percent in length errors,
  -- which are second order (overlap is first order)
  -- ac.debug('look-up overlap',   dot(neck.look, neck.up))
  -- ac.debug('look-side overlap', dot(neck.look, neck.side))
  -- ac.debug('side-up overlap',   dot(neck.side, neck.up))

end

-- 2D vector cross product of vector (x1,y1) and (x2,y2). Returns a positive or negative scalar.
function cross2d(x1,y1,x2,y2)
  return y1*x2 - x1*y2
end



