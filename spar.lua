local Pine3D = require("Pine3D")

local ThreeDFrame = Pine3D.newFrame(_, _, _, _, true)
ThreeDFrame:setCamera(-8, 0.6, 0)
ThreeDFrame:setFoV(90)

local pineObjects = {}

Vec3 = {}
Vec3.__index = Vec3

function Vec3.new(x, y, z)
    return setmetatable({x = x or 0, y = y or 0, z = z or 0}, Vec3)
end

function Vec3:add(v)
    return Vec3.new(self.x + v.x, self.y + v.y, self.z + v.z)
end

function Vec3:sub(v)
    return Vec3.new(self.x - v.x, self.y - v.y, self.z - v.z)
end

function Vec3:scale(s)
    return Vec3.new(self.x * s, self.y * s, self.z * s)
end

function Vec3:length()
    return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)
end

function Vec3:dot(v)
    return self.x * v.x + self.y * v.y + self.z * v.z
end

function Vec3:cross(v)
    return Vec3.new(
        self.y * v.z - self.z * v.y,
        self.z * v.x - self.x * v.z,
        self.x * v.y - self.y * v.x
    )
end

function Vec3:normalize()
    local len = self:length()
    if len == 0 then return Vec3.new(0, 0, 0) end
    return self:scale(1 / len)
end

AABB = {}
AABB.__index = AABB

function AABB.new(min, max)
    return setmetatable({min = min, max = max}, AABB)
end

function AABB:checkCollision(other)
    return (self.min.x <= other.max.x and self.max.x >= other.min.x) and
           (self.min.y <= other.max.y and self.max.y >= other.min.y) and
           (self.min.z <= other.max.z and self.max.z >= other.min.z)
end

PhysicsObject = {}
PhysicsObject.__index = PhysicsObject

function PhysicsObject.new(opts)
    local position = opts.position
    local size = opts.size
    local mass = opts.mass
    local compressed = opts.compressed or false
    local model = opts.model

    local halfSize = size:scale(0.5)
    if compressed then
        table.insert(pineObjects, ThreeDFrame:newCompressedObject(model, position.x, position.y, position.z))
    else
        table.insert(pineObjects, ThreeDFrame:newObject(model, position.x, position.y, position.z))
    end
    local obj = setmetatable({
        pineObject = #pineObjects,
        position = position,
        velocity = Vec3.new(0, 0, 0),
        rotation = Vec3.new(0, 0, 0),
        rotation_velocity = Vec3.new(0, 0, 0),
        force = Vec3.new(0, 0, 0),
        mass = mass or 1,
        size = size,
        drag = opts.drag,
        lifetime = opts.lifetime or nil,
        age = 0,
        aabb = AABB.new(position:sub(halfSize), position:add(halfSize))
    }, PhysicsObject)
    return obj
end

function PhysicsObject:applyForce(force)
    self.force = self.force:add(force)
end

function PhysicsObject:update(dt)
    self.age = self.age + dt
    local acceleration = self.force:scale(1 / self.mass)

    self.velocity = self.velocity:add(acceleration:scale(dt))

    self.position = self.position:add(self.velocity:scale(dt))

    self.rotation = self.rotation:add(self.rotation_velocity:scale(dt))

    local halfSize = self.size:scale(0.5)
    self.aabb.min = self.position:sub(halfSize)
    self.aabb.max = self.position:add(halfSize)

    self.force = Vec3.new(0, 0, 0)
end

function PhysicsObject:applyGravity(g, dt)
    local gravityAcceleration = Vec3.new(0, g, 0)
    self.velocity = self.velocity:add(gravityAcceleration:scale(dt))
end


local function getCollisionMTV(aabb1, aabb2)
    local dx1 = aabb2.max.x - aabb1.min.x
    local dx2 = aabb1.max.x - aabb2.min.x
    local dy1 = aabb2.max.y - aabb1.min.y
    local dy2 = aabb1.max.y - aabb2.min.y
    local dz1 = aabb2.max.z - aabb1.min.z
    local dz2 = aabb1.max.z - aabb2.min.z

    local overlaps = {
        {axis = Vec3.new(1,0,0), depth = dx1},
        {axis = Vec3.new(-1,0,0), depth = dx2},
        {axis = Vec3.new(0,1,0), depth = dy1},
        {axis = Vec3.new(0,-1,0), depth = dy2},
        {axis = Vec3.new(0,0,1), depth = dz1},
        {axis = Vec3.new(0,0,-1), depth = dz2},
    }

    local minOverlap = nil
    for _, overlap in ipairs(overlaps) do
        if overlap.depth > 0 then
            if minOverlap == nil or overlap.depth < minOverlap.depth then
                minOverlap = overlap
            end
        end
    end

    return minOverlap and minOverlap.axis:scale(minOverlap.depth) or Vec3.new(0,0,0)
end

local function resolveCollision(obj1, obj2)
    local mtv = getCollisionMTV(obj1.aabb, obj2.aabb)
    if mtv:length() == 0 then return end

    local totalMass = obj1.mass + obj2.mass
    obj1.position = obj1.position:add(mtv:scale(obj2.mass / totalMass))
    obj2.position = obj2.position:sub(mtv:scale(obj1.mass / totalMass))

    local halfSize1 = obj1.size:scale(0.5)
    obj1.aabb.min = obj1.position:sub(halfSize1)
    obj1.aabb.max = obj1.position:add(halfSize1)

    local halfSize2 = obj2.size:scale(0.5)
    obj2.aabb.min = obj2.position:sub(halfSize2)
    obj2.aabb.max = obj2.position:add(halfSize2)

    local normal = mtv:normalize()
    local relVel = obj1.velocity:sub(obj2.velocity)
    local velAlongNormal = relVel:dot(normal)

    if velAlongNormal > 0 then
        return
    end

    local restitution = 0.5
    local friction = 0.3

    local j = -(1 + restitution) * velAlongNormal
    j = j / (1 / obj1.mass + 1 / obj2.mass)
    local impulse = normal:scale(j)

    obj1.velocity = obj1.velocity:add(impulse:scale(1 / obj1.mass))
    obj2.velocity = obj2.velocity:sub(impulse:scale(1 / obj2.mass))

    local tangent = relVel:sub(normal:scale(velAlongNormal))
    if tangent:length() > 0 then
        tangent = tangent:normalize()
        local jt = -relVel:dot(tangent)
        jt = jt / (1 / obj1.mass + 1 / obj2.mass)

        local frictionImpulse
        if math.abs(jt) < j * friction then
            frictionImpulse = tangent:scale(jt)
        else
            frictionImpulse = tangent:scale(-j * friction)
        end

        obj1.velocity = obj1.velocity:add(frictionImpulse:scale(1 / obj1.mass))
        obj2.velocity = obj2.velocity:sub(frictionImpulse:scale(1 / obj2.mass))

        local I1 = (1/6) * obj1.mass * obj1.size:length()^2
        local I2 = (1/6) * obj2.mass * obj2.size:length()^2
        local torque1 = frictionImpulse:cross(normal):scale(1 / I1)
        local torque2 = frictionImpulse:cross(normal):scale(1 / I2)
        obj1.rotation_velocity = obj1.rotation_velocity:add(torque1)
        obj2.rotation_velocity = obj2.rotation_velocity:sub(torque2)
    end
end

local objects = {}

local function gameUpdate(dt)
    local collisionedPairs = {}

    for i = 1, #objects do
        local obj = objects[i]
        obj:applyGravity(-9.8, dt)
        local v = obj.velocity
        obj.velocity.x = v.x - obj.drag * v.x * math.abs(v.x) * dt
        obj.velocity.y = v.y - obj.drag * v.y * math.abs(v.y) * dt
        obj.velocity.z = v.z - obj.drag * v.z * math.abs(v.z) * dt

        local dragFactor = 1 - obj.drag * dt
        dragFactor = math.max(0, dragFactor)
        obj.rotation_velocity.x = obj.rotation_velocity.x * dragFactor
        obj.rotation_velocity.y = obj.rotation_velocity.y * dragFactor
        obj.rotation_velocity.z = obj.rotation_velocity.z * dragFactor

        for j = i + 1, #objects do
            local obj2 = objects[j]
            local key = tostring(obj) .. "-" .. tostring(obj2)

            if not collisionedPairs[key] and obj.aabb:checkCollision(obj2.aabb) then
                resolveCollision(obj, obj2)
                collisionedPairs[key] = true
            end
        end
    end

    local i = 1
    while i <= #objects do
        local obj = objects[i]
        obj:update(dt)
        if obj.lifetime and obj.age >= obj.lifetime then
            pineObjects[obj.pineObject]:setModel({x1 = 0, x2 = 0, x3 = 0, y1 = 0, y2 = 0, y3 = 0, z1 = 0, z2 = 0, z3 = 0, c = 0})
            table.remove(objects, i)
        else
            pineObjects[obj.pineObject]:setPos(obj.position.x, obj.position.y, obj.position.z)
            pineObjects[obj.pineObject]:setRot(obj.rotation.x, obj.rotation.y, obj.rotation.z)
            i = i + 1
        end
    end

    ThreeDFrame:drawObjects(pineObjects)
    ThreeDFrame:drawBuffer()
end

local obj1 = PhysicsObject.new({
    model = "models/pineapple",
    position = Vec3.new(0, 2, 0),
    size = Vec3.new(1, 1, 1),
    mass = 2,
    drag = 0.1,
    lifetime = 0.25
})
local obj2 = PhysicsObject.new({
    model = "models/pineapple",
    position = Vec3.new(0, 7, 5),
    size = Vec3.new(1, 1, 1),
    mass = 2,
    drag = 0.1,
    lifetime = 2
})

obj1.velocity = Vec3.new(0, 0, 0)
obj2.velocity = Vec3.new(0, -10, -10)
obj2.rotation_velocity = Vec3.new(0, 0, 10)

table.insert(objects, obj1)
table.insert(objects, obj2)

local dt = 0.016

local function gameLogic()
    local start = os.epoch() / 100000
    while true do
        if (os.epoch() / 100000) - start > 0.1 then
            start = os.epoch() / 100000
            local newObj1 = PhysicsObject.new({
                model = "models/pineapple",
                position = Vec3.new(0, 2, 0),
                size = Vec3.new(1, 1, 1),
                mass = 2,
                drag = 0.1,
                lifetime = 2
            })
            local newObj2 = PhysicsObject.new({
                model = "models/pineapple",
                position = Vec3.new(0, 7, 5),
                size = Vec3.new(1, 1, 1),
                mass = 2,
                drag = 0.1,
                lifetime = 2
            })

            newObj1.velocity = Vec3.new(0, 0, 0)
            newObj2.velocity = Vec3.new(0, -10, -10)
            newObj2.rotation_velocity = Vec3.new(0, 0, 10)

            table.insert(objects, newObj1)
            table.insert(objects, newObj2)
        end
        gameUpdate(dt)
        sleep(dt)
    end
end

gameLogic()
