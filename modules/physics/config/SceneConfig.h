#pragma once

#include "types/Type.h"

#include <map>
#include <string>
#include <nlohmann/json.hpp>

#include <GL/gl.h>
#include <iostream>

NLOHMANN_JSON_SERIALIZE_ENUM( ObjectType, {
    {ObjectType::Particle, "Particle"},
    {ObjectType::Cube2d2, "Cube2d2"},
    {ObjectType::Cube2d3, "Cube2d3"},
    {ObjectType::Cube3d2, "Cube3d2"},
})

struct Bound
{
    Vec3 min;
    Vec3 max;

    inline Bound& operator *=(const float v)
    {
        min.x *= v;
        min.y *= v;
        min.z *= v;

        max.x *= v;
        max.y *= v;
        max.z *= v;

        return *this;
    }
};

struct ObjectConfig
{
    ObjectType type;
    float radius;
    float mass;
    float bounce;
    float damping;
    float tanVel;
    uint count;
    bool selfCollision;
    Spring spring;

    GLuint vboVertices;
    GLuint vboColor;
};

class SceneConfig
{
public:
    static SceneConfig Parse(const nlohmann::json& data)
    {
        SceneConfig result;
        result.gravity = data["gravity"];
        result.deltaTime = data["delta_time"];
        result.maxSpeed = data["max_speed"];
        
        result.bound.min.x = data["bounds"]["min"]["x"];
        result.bound.min.y = data["bounds"]["min"]["y"];
        result.bound.min.z = data["bounds"]["min"]["z"];
        result.bound.max.x = data["bounds"]["max"]["x"];
        result.bound.max.y = data["bounds"]["max"]["y"];
        result.bound.max.z = data["bounds"]["max"]["z"];

        const auto objConfigs = data["objects"];
        
        for (const auto& objConfig : objConfigs)
        {
            ObjectConfig obj;
            obj.type = objConfig["type"];
            obj.radius = objConfig["radius"];
            obj.mass = objConfig["mass"];
            obj.bounce = objConfig["bounce"];
            obj.damping = objConfig["damping"];
            obj.tanVel = objConfig["tan_vel"];
            obj.count = objConfig["count"];
            obj.selfCollision = objConfig["self_collision"];

            if (objConfig.contains("spring"))
            {
                obj.spring.k = objConfig["spring"]["k"];
                obj.spring.damping = objConfig["spring"]["damping"];
                obj.spring.length = objConfig["spring"]["length"];

                std::cout << "LEN: " << obj.spring.length << std::endl;
            }
            else if (obj.type != ObjectType::Particle)
            {
                throw "Spring is not specified\n";
            }

            result.objects.push_back(obj);
        }

        if (data.contains("command"))
        {
            result.command.type = data["command"]["type"];
            result.command.index = data["command"]["index"];
            result.command.pos.x = data["command"]["pos"]["x"];
            result.command.pos.y = data["command"]["pos"]["y"];
            result.command.pos.z = data["command"]["pos"]["z"];
        }

        return result;
    }

// todo: private
public:
    float gravity;
    float deltaTime;
    float maxSpeed;
    Bound bound;

    std::vector<ObjectConfig> objects;
    Command command;
};