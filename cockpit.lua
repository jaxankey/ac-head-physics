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
  PITCH_SCALE     = 1,       -- Level of car pitch tracking: 1 to follow car, 0 to follow horizon
  PITCH_FREQUENCY = 3,
  PITCH_DAMPING   = 1,
  PITCH_LIMIT     = 30,   -- degrees

  -- Head rolling with car
  ROLL_SCALE     = 1,       -- Level of car roll tracking:  1 to follow car, 0 to follow horizon
  ROLL_FREQUENCY = 3,
  ROLL_DAMPING   = 1,
  ROLL_LIMIT     = 40,      -- degrees

  -- Let's the car turn under your head to kill yaw wobbles (yawbbles). Takes some getting used to!
  YAW_SCALE     = 0.03,   -- How far the car turns under your head for a given rotation speed, and the maximum wobble amplitude you can eliminate.
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
function soft_clip_less_linear(value, max)
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


-- Dyanmical variables for calculating accelration transients
local transient = {
  x     = FilterHighPass:new(settings.HORIZONTAL_RECENTER_TIME),
  y     = FilterHighPass:new(settings.VERTICAL_RECENTER_TIME),
  z     = FilterHighPass:new(settings.FORWARD_RECENTER_TIME),
  yaw   = FilterHighPass:new(settings.YAW_SCALE),
}

-- Precomputed scale factors (doesn't save much time but oh well)
local horizontal_scale = settings.HORIZONTAL_SCALE*settings.POSITION_SCALE
local vertical_scale   = settings.VERTICAL_SCALE  *settings.POSITION_SCALE
local forward_scale    = settings.FORWARD_SCALE   *settings.POSITION_SCALE

-- For parsing lines like SETTING=THING
function split_and_strip(input, delimiter)
  local result = {}

  -- Loop over the tokens using this archaic bullshit string parsing.
  for token in input:gmatch("([^" .. delimiter .. "]+)") do
      -- Strip leading and trailing whitespaces from each token
      local strippedToken = token:match("^%s*(.-)%s*$")
      table.insert(result, strippedToken)
  end

  return result
end

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

-- Does an infinitesimal / approximate rotation of unit vector (v) about the 
-- supplied axis unit vector (a) by a distance (d << pi)
function small_rotation(v, a, d)
  return v + d*cross(a,v)
end

-- Safely normalizes, making sure it's not dividing by zero.
function safe_normalize(v)
  if v:lengthSquared() < 1e-20 then v = v+vec3(1e-10,0,0) end
  return v:normalize()
end

-- Jack: This script will wig out when there are frame drops, and may produce
--       different effects for people with different frame rates. It doesn't use dt.
local last_car_look = vec3(0,0,0)
function script.update(dt, mode, turnMix)

  -- The angles we will eventually add to the neck vectors
  local pitch = 0
  local roll  = 0
  local yaw   = 0

  -- If we just jumped to a new location or we're going slow, reset stuff
  if car.justJumped or car.speedMs < 0.1 then
    head.pitch:reset()
    head.roll:reset()
    head.yaw:reset()
    head.x:reset()
    head.y:reset()
    head.z:reset()

    transient.x:reset()
    transient.y:reset()
    transient.z:reset()
    transient.yaw:reset()

    -- Align with the car
    head.pitch.value = car.look.y
    head.roll.value  = car.side.y

    -- Set it up to do the correct rotation.
    pitch =  car.look.y - head.pitch.value*settings.PITCH_SCALE
    roll  = -car.side.y + head.roll .value*settings.ROLL_SCALE

    -- Reset the last car values so the car isn't thought to be rotating
    last_car_look = car.look:clone()
    
    --ac.debug('state', 'stop')
    return
  else
    --ac.debug('state', 'moving')

    -- Displacement physics
    -- 
    -- For each of these, high-pass filter the car acceleration to get transients,
    -- then evolve the position under this acceleration and displace the head.
    --
    -- Note car.acceleration (unlike rotation velocity?) seems to have its axes aligned with the car
    --  x = car.side
    --  y = car.up
    --  z = car.look
    if settings.HORIZONTAL_ENABLED == 1 then
      transient.x:evolve(dt, car.acceleration.x)
      head.x:evolve_acceleration(dt, horizontal_scale*transient.x.value)
      neck.position = neck.position - head.x.value*car.side
    end
    if settings.VERTICAL_ENABLED == 1 then
      transient.y:evolve(dt, car.acceleration.y)
      head.y:evolve_acceleration(dt, vertical_scale*transient.y.value)
      neck.position = neck.position - head.y.value*car.up
    end
    if settings.FORWARD_ENABLED == 1 then
      transient.z:evolve(dt, car.acceleration.z)
      head.z:evolve_acceleration(dt, forward_scale*transient.z.value)
      neck.position = neck.position - head.z.value*car.look
    end

    -- neck.look, neck.up, neck.side, car.look, car.up, car.side are orthogonal unit vectors
    -- for the orientation of the car and neck (matched to each other at the start of this 
    -- update function).

    -- Pitch dynamics: Evolve toward the car's look.y (vertical) value, and add the difference
    if settings.PITCH_ENABLED == 1 then
      pitch = car.look.y - head.pitch:evolve_target(dt,car.look.y)*settings.PITCH_SCALE
    end

    -- Roll dynamics: Evolve toward the car's side.y (vertical) value, and add the difference
    if settings.ROLL_ENABLED == 1 then
      roll = -car.side.y + head.roll:evolve_target(dt,car.side.y)*settings.ROLL_SCALE
    end

    -- Yaw dynamics
    if settings.YAW_ENABLED == 1 then
      -- 1. Add the angular change to the yaw value, but high-pass 
      --    it so that we are sensitive only to transients. Note we go through 
      --    the /dt process so that variable frame rate has a smoother evolution.
      -- Note that with the transient disabled, this just accumulates the total angle
      --ac.debug('test', vec2(car.look.x-last_car_look.x, car.look.z-last_car_look.z):length())
      head.yaw.value = head.yaw.value + transient.yaw:evolve(dt,
        cross2d(car.look.x, car.look.z, last_car_look.x, last_car_look.z)/dt) * dt
      --local dyaw = cross2d(car.look.x, car.look.z, last_car_look.x, last_car_look.z)
      --head.yaw.value = head.yaw.value + dyaw
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

  -- Remember the last car look
  last_car_look = car.look:clone()

  -- Do the rotations
  -- JACK: Do the rotations for all 3 and rotate by the angle
  --       rather than distance to handle non-orthogonality?
  neck.look = small_rotation(neck.look, car.side, pitch)
  neck.up   = small_rotation(neck.up  , car.side, pitch)
  neck.look = small_rotation(neck.look, car.up  , yaw  )
  neck.side = small_rotation(neck.side, car.up  , yaw  )
  neck.up   = small_rotation(neck.up  , car.look, roll )
  neck.side = small_rotation(neck.side, car.look, roll )


  -- These tests showed me that even crashing an F2004 the lengths changed by at most ~2% from unity.
  -- So the small rotation approximation is good
  --ac.debug('look.length', neck.look:length())
  --ac.debug('up.length', neck.up:length())
  --ac.debug('side.length', neck.side:length())

  -- These tests showed that normal driving lead to <1% overlap between these basis vectors.
  -- In a crash, these things can be like 0.2, which is consistent with few percent in length errors,
  -- which are second order (overlap is first order)
  -- ac.debug('look-up overlap',   dot(neck.look, neck.up))
  -- ac.debug('look-side overlap', dot(neck.look, neck.side))
  -- ac.debug('side-up overlap',   dot(neck.side, neck.up))

end

-- VERY rough normalize function that is FAST and gets a 
-- length between 1 and 1.73. It won't run away if we do this 
-- a lot. :)
function rough_normalize(vector)
  return vector / math.max(vector.x, vector.y, vector.z)
end

-- Vector rotation (not used right now but I don't want to redo it. :)
function rotate(vector, axis, angle)
  local cosAngle = math.cos(angle)
  local sinAngle = math.sin(angle)
  return vector * cosAngle + cross(axis, vector) * sinAngle + axis * dot(axis, vector) * (1 - cosAngle)
end

-- Cross product of two 3d vectors
function cross(v1, v2)
  return vec3(v1.y*v2.z - v1.z*v2.y, v1.z*v2.x - v1.x*v2.z, v1.x*v2.y - v1.y*v2.x)
end

-- 2D vector cross product of vector (x1,y1) and (x2,y2). Returns a positive or negative scalar.
function cross2d(x1,y1,x2,y2)
  return y1*x2 - x1*y2
end

-- Dot product of two 3d vectors.
function dot(v1, v2)
  return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z
end



