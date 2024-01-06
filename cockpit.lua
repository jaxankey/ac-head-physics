local scriptSettings = ac.INIConfig.scriptSettings()
local settings = scriptSettings:mapSection('SETTINGS', {

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

  -- Left-right motion (Roxbury axis)
  HORIZONTAL_SCALE  = 1,
  HORIZONTAL_LIMIT  = 0.05,
  HORIZONTAL_FREQUENCY = 1.0,
  HORIZONTAL_DAMPING   = 1,
  HORIZONTAL_RECENTER_TIME = 2,

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

  -- Head rolling with car
  ROLL_SCALE     = 1,       -- Level of car roll tracking:  1 to follow car, 0 to follow horizon
  ROLL_FREQUENCY = 3,
  ROLL_DAMPING   = 1,

  -- Let's the car turn under your head to kill yaw wobbles (yawbbles). Takes some getting used to!
  YAW_SCALE     = 0.03, -- How far the car turns under your head for a given rotation speed, and the maximum wobble amplitude you can eliminate.
  YAW_FREQUENCY = 0.5,    -- Filter frequency for wobbles (faster wobbles are reduced). The puke-wobbles are often ~1-2 Hz.
  YAW_DAMPING   = 1,
})



-- Constants
local pi = 3.1415926535
local pi24 = 4*pi*pi


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
    instance.frequency = frequency
    instance.damping   = damping
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

    -- Update the position with this velocity
    self.value = self.value + self.velocity*dt

    -- May as well return it
    return self.value
end

-- Function to evolve using an applied acceleration.
-- dt is the time step in seconds, assumed to be small.
function FilterSpring:evolve_acceleration(dt, a)

  -- The target in this case is always zero.
  self.target = 0

  -- Get the deviation from the target
  local dx = self.value - self.target
  local v  = self.velocity
  local f  = self.frequency
  local g  = self.damping
  local l  = 0

  -- Update the velocity from the acceleration (damping and spring)
  if l > 0 then
    self.velocity = v + (a - 4*pi*g*f*v - pi24*f*f*dx*(1+dx*dx/(l*l)))*dt
  else
    self.velocity = v + (a - 4*pi*g*f*v - pi24*f*f*dx)*dt
  end
  -- Update the position with this velocity
  self.value = self.value + self.velocity*dt

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


-- Soft clipping function to place bounds on any value
--  value is the input value, and the return value will
--  be bounded between +/-max
function soft_clip(value, max)
  -- return value*max/(1+math.abs(value))
  if value >  2*max then return  max end
  if value < -2*max then return -max end
  return value - value*math.abs(value)/(4*max)
end


-- Dynamical variables for head position
local head = {
  x     = FilterSpring:new(settings.HORIZONTAL_FREQUENCY, settings.HORIZONTAL_DAMPING, settings.HORIZONTAL_LIMIT),
  y     = FilterSpring:new(settings.VERTICAL_FREQUENCY  , settings.VERTICAL_DAMPING  , settings.VERTICAL_LIMIT),
  z     = FilterSpring:new(settings.FORWARD_FREQUENCY   , settings.FORWARD_DAMPING   , settings.FORWARD_LIMIT),
  pitch = FilterSpring:new(settings.PITCH_FREQUENCY     , settings.PITCH_DAMPING     ),
  roll  = FilterSpring:new(settings.ROLL_FREQUENCY      , settings.ROLL_DAMPING      ),
  yaw   = FilterSpring:new(settings.YAW_FREQUENCY       , settings.YAW_DAMPING       ),
}


-- Dyanmical variables for calculating accelration transients
local transient = {
  x     = FilterHighPass:new(settings.HORIZONTAL_RECENTER_TIME),
  y     = FilterHighPass:new(settings.VERTICAL_RECENTER_TIME),
  z     = FilterHighPass:new(settings.FORWARD_RECENTER_TIME),
  yaw   = FilterHighPass:new(settings.YAW_SCALE),
}

-- starting and current head position relative to car
local p0w = vec3(0,0,0) -- Starting head position in world coordinates (calcualted / updated every update)
local pw  = vec3(0,0,0) -- Current head position in car coordinates (caluculate / updated every update)
local eye_pitch = 0     -- Pitch zero value


-- Car's side and up vector without any roll around look
local car_side_no_roll = vec3(1,0,0)
local car_up_no_roll   = vec3(0,1,0)

-- Precomputed scale factors
local horizontal_scale = settings.HORIZONTAL_SCALE*settings.POSITION_SCALE
local vertical_scale   = settings.VERTICAL_SCALE  *settings.POSITION_SCALE
local forward_scale    = settings.FORWARD_SCALE   *settings.POSITION_SCALE

-- multipurpose
local temp = 0

-- For parsing lines like SETTING=THING
function split_and_strip(input, delimiter)
  local result = {}

  for token in input:gmatch("([^" .. delimiter .. "]+)") do
      -- Strip leading and trailing whitespaces from each token
      local strippedToken = token:match("^%s*(.-)%s*$")
      table.insert(result, strippedToken)
  end

  return result
end

-- To get the eye pitch setting we look at a file
local path_view = os.getenv('HOMEPATH') .. '\\Documents\\Assetto Corsa\\cfg\\cars\\' .. ac.getCarID() .. '\\view.ini'
local last_check = os.clock()
function get_eye_pitch()
  
  -- Used by line loop
  local result

  -- Open the file in read mode
  local file = io.open(path_view, "r")

  -- Check if the file was opened successfully
  if file then

      -- Iterate over each line in the file
      for line in file:lines() do

        -- if the line is interesting, look for the pitch angle
        if string.len(line) > 0 then

            -- Get the key-value pair
            result = split_and_strip(line, '=')

            -- Check for the key we care about.
            if string.match(result[1], 'ON_BOARD_PITCH_ANGLE') then 
              file:close()
              return tonumber(result[2])
            end
          end
      end

      -- Close the file when done
      file:close()
      return true
  else
      return false
  end

end

-- Jack: This script will wig out when there are frame drops, and may produce
--       different effects for people with different frame rates. It doesn't use dt.
function script.update(dt, mode, turnMix)
  
  -- IDEAS
    -- car.isInPit true when the car is physically in the pitbox
    -- car.isActive at least true when on track
    -- car.isUserControlled

  -- The only variable I can work with is neck.position, the 
  -- absolute position on the map. Even subtracting car.position
  -- doesn't give a constant vector as the car rotates, 
  -- meaning we gotta do some dot products.

  -- Get some vectors used by yaw and roll
  if settings.ROLL_ENABLED == 1 or settings.YAW_ENABLED == 1 then

    -- Vector pointing perpendicular to the car's forward direction in the xz plane
    -- Could reduce computation here by using only car.look.x and car.look.z, but this will be minimal compared
    -- to normalizing / square roots
    car_side_no_roll = cross(car.look, vec3(0,1,0))

    -- This could be zero length if the car is pointing straight up, so check for that before normalizing
    if car_side_no_roll:lengthSquared() < 1e-20 then car_side_no_roll = car_side_no_roll + vec3(1e-10,0,0) end

    -- normalize it so the length isn't changing during dynamics
    car_side_no_roll = car_side_no_roll:normalize()

    -- normalized up vector in the absence of roll for the other roll component
    car_up_no_roll = cross(car_side_no_roll, car.look):normalize()

  end

  -- If we're stopped, just store the values and quit
  -- We make a manual copy here to avoid referencing by address issues
  if  car.velocity.x^2 + car.velocity.y^2 + car.velocity.z^2 < 1e-20
  and head.x.value^2   + head.y.value^2   + head.z.value^2   < 1e-10
  or car.justJumped then

    -- Reset the transient filters
    transient.x:reset()
    transient.y:reset()
    transient.z:reset()
    transient.yaw:reset()

    -- Reset the head response filters
    head.x:reset()
    head.y:reset()
    head.z:reset()
    head.pitch:reset()
    head.roll :reset()
    head.roll.value = dot(car_side_no_roll, car.up)
    head.yaw:reset()

    -- IMPORTANT: Set the head's current displacement from car center, world coordinates
    --pw = neck.position - car.position

    -- Set the "zero" displacement in car coordinates
    --p0.x = dot(pw, car.side)
    --p0.y = dot(pw, car.up  )
    --p0.z = dot(pw, car.look)

    -- Every quarter second open the ini file to check the eye pitch
    if os.clock() - last_check > 0.25 then 
      temp = get_eye_pitch()
      if type(temp) == 'number' then eye_pitch = temp end
      last_check = os.clock()
    end

    -- reset the neck alignment with the car
    neck.look.x = car.look.x
    neck.look.y = car.look.y + eye_pitch
    neck.look.z = car.look.z

    return
  end

  ---------------------------------------------------------
  -- Pitch, yaw, and roll

  -- Get the vertical (y) component of the car.look variable to 
  -- find the desired equilibrium, and plug this into the 
  -- harmonic oscillator for rotation
  if settings.PITCH_ENABLED == 1 then neck.look.y = eye_pitch + settings.PITCH_SCALE*head.pitch:evolve_target(dt, car.look.y) end

  -- Reduce yaw wobble (yawbble), which occurs as tires grip in and out, which makes me sick in VR
  if settings.YAW_ENABLED == 1 then

      -- High-pass the angular velocity so that we are sensitive only to transients. 
    -- Add this to the existing yaw, such that a given angular velocity produces
    -- a fixed yaw. Note we go through the /dt process so that variable frame rate 
    -- has a smoother evolution.
    head.yaw.value = head.yaw.value + transient.yaw:evolve(dt, -car.angularVelocity.y) * dt
    
    -- Now have the head yaw spring try to keep up
    head.yaw:evolve_target(dt, 0)
    
    -- Now add this perpendicularly to the car look to get the head location
    neck.look.x = neck.look.x - head.yaw.value*car_side_no_roll.x
    neck.look.z = neck.look.z - head.yaw.value*car_side_no_roll.z
  end

  -- To figure out the roll angle about the car's look axis, we need to dot the up vector with an 
  -- in-plane (no roll) side vector.
  if settings.ROLL_ENABLED == 1 then

    -- The head roll component is then the (lagging) dot product with car_side_no_roll
    head.roll:evolve_target(dt, dot(car_side_no_roll, car.up))

    -- Now reconstruct the head's (lagging) up vector
    temp = head.roll.value*settings.ROLL_SCALE -- side value
    neck.up = car_side_no_roll*temp + car_up_no_roll*math.sqrt(1-temp*temp)

  else
    neck.up = vec3(car.up.x, car.up.y, car.up.z) -- make a copy
  end

  --------------------------------------------------------
  -- car.acceleration and displacement
    -- x: leftward relative to car.look
    -- y: vertical
    -- z: along car.look
    -- Magnitude generally ~1

  -- High-pass filter the car acceleration to get transients
  if settings.HORIZONTAL_ENABLED == 1 then
    transient.x:evolve(dt, car.acceleration.x)
    head.x:evolve_acceleration(dt, horizontal_scale*transient.x.value)
  else
    head.x.value = 0
  end
  if settings.VERTICAL_ENABLED == 1 then
    transient.y:evolve(dt, car.acceleration.y)
    head.y:evolve_acceleration(dt, vertical_scale*transient.y.value)
  else
    head.y.value = 0
  end
  if settings.FORWARD_ENABLED == 1 then
    transient.z:evolve(dt, car.acceleration.z)
    head.z:evolve_acceleration(dt, forward_scale*transient.z.value)
  else
    head.z.value = 0
  end

  -- Equilbrium head displacement from car in world coordinates
  p0w = car.driverEyesPosition.x*car.side + car.driverEyesPosition.y*car.up + car.driverEyesPosition.z*car.look + car.position

  -- Convert the dynamical neck displacments into world coordinates
  pw = - soft_clip(head.x.value, settings.HORIZONTAL_LIMIT)*car.side
       - soft_clip(head.y.value, settings.VERTICAL_LIMIT  )*car.up
       - soft_clip(head.z.value, settings.FORWARD_LIMIT   )*car.look

  -- Get the new neck position
  neck.position = p0w + pw

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



