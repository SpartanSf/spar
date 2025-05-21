local Pine3D = require("Pine3D")

local ThreeDFrame = Pine3D.newFrame()
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

function PhysicsObject.new(position, size, mass)
    local halfSize = size:scale(0.5)
    table.insert(pineObjects, ThreeDFrame:newObject("models/pineapple", position.x, position.y, position.z))
    local obj = setmetatable({
        pineObject = #pineObjects,
        position = position,
        velocity = Vec3.new(0, 0, 0),
        force = Vec3.new(0, 0, 0),
        mass = mass or 1,
        size = size,
        aabb = AABB.new(position:sub(halfSize), position:add(halfSize))
    }, PhysicsObject)
    return obj
end

function PhysicsObject:applyForce(force)
    self.force = self.force:add(force)
end

function PhysicsObject:update(dt)
    local acceleration = self.force:scale(1 / self.mass)

    self.velocity = self.velocity:add(acceleration:scale(dt))

    self.position = self.position:add(self.velocity:scale(dt))

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
    local velAlongNormal = relVel.x * normal.x + relVel.y * normal.y + relVel.z * normal.z

    if velAlongNormal > 0 then
        return
    end

    local restitution = 0.5
    local j = -(1 + restitution) * velAlongNormal
    j = j / (1 / obj1.mass + 1 / obj2.mass)

    local impulse = normal:scale(j)
    obj1.velocity = obj1.velocity:add(impulse:scale(1 / obj1.mass))
    obj2.velocity = obj2.velocity:sub(impulse:scale(1 / obj2.mass))
end



local obj1 = PhysicsObject.new(Vec3.new(0, 0, 0), Vec3.new(1, 1, 1), 2)
local obj2 = PhysicsObject.new(Vec3.new(0, 5, 5), Vec3.new(1, 1, 1), 2)

obj1.velocity = Vec3.new(0, 0, 0)
obj2.velocity = Vec3.new(0, -10, -10)

local dt = 0.016


while true do
    if obj1.aabb:checkCollision(obj2.aabb) then
        resolveCollision(obj1, obj2)
    end

    obj1:applyGravity(-9.8, dt)
    obj2:applyGravity(-9.8, dt)

    obj1:update(dt)
    obj2:update(dt)

    pineObjects[obj1.pineObject]:setPos(obj1.position.x, obj1.position.y, obj1.position.z)
    pineObjects[obj2.pineObject]:setPos(obj2.position.x, obj2.position.y, obj2.position.z)

    ThreeDFrame:drawObjects(pineObjects)
    ThreeDFrame:drawBuffer()

    sleep(dt)
end
