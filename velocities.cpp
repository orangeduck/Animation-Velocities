extern "C"
{
#include "raylib.h"
#define RAYMATH_STATIC_INLINE
#include "raymath.h"
#include "rlgl.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
}
#if defined(PLATFORM_WEB)
#include <emscripten/emscripten.h>
#endif

#include "common.h"
#include "vec.h"
#include "quat.h"
#include "spring.h"
#include "array.h"
#include "character.h"
#include "database.h"

#include <initializer_list>
#include <vector>
#include <functional>
#include <string>

//--------------------------------------

static inline Vector3 to_Vector3(vec3 v)
{
    return (Vector3){ v.x, v.y, v.z };
}

//--------------------------------------

void deform_character_mesh(
  Mesh& mesh, 
  const character& c,
  const slice1d<vec3> bone_anim_positions,
  const slice1d<quat> bone_anim_rotations,
  const slice1d<vec3> bone_anim_scales,
  const slice1d<int> bone_parents)
{
    linear_blend_skinning_positions(
        slice1d<vec3>(mesh.vertexCount, (vec3*)mesh.vertices),
        c.positions,
        c.bone_weights,
        c.bone_indices,
        c.bone_rest_positions,
        c.bone_rest_rotations,
        bone_anim_positions,
        bone_anim_rotations,
        bone_anim_scales);
    
    linear_blend_skinning_normals(
        slice1d<vec3>(mesh.vertexCount, (vec3*)mesh.normals),
        c.normals,
        c.bone_weights,
        c.bone_indices,
        c.bone_rest_rotations,
        bone_anim_rotations);
    
    UpdateMeshBuffer(mesh, 0, mesh.vertices, mesh.vertexCount * 3 * sizeof(float), 0);
    UpdateMeshBuffer(mesh, 2, mesh.normals, mesh.vertexCount * 3 * sizeof(float), 0);
}

Mesh make_character_mesh(character& c)
{
    Mesh mesh = { 0 };
    
    mesh.vertexCount = c.positions.size;
    mesh.triangleCount = c.triangles.size / 3;
    mesh.vertices = (float*)MemAlloc(c.positions.size * 3 * sizeof(float));
    mesh.texcoords = (float*)MemAlloc(c.texcoords.size * 2 * sizeof(float));
    mesh.normals = (float*)MemAlloc(c.normals.size * 3 * sizeof(float));
    mesh.indices = (unsigned short*)MemAlloc(c.triangles.size * sizeof(unsigned short));
    
    memcpy(mesh.vertices, c.positions.data, c.positions.size * 3 * sizeof(float));
    memcpy(mesh.texcoords, c.texcoords.data, c.texcoords.size * 2 * sizeof(float));
    memcpy(mesh.normals, c.normals.data, c.normals.size * 3 * sizeof(float));
    memcpy(mesh.indices, c.triangles.data, c.triangles.size * sizeof(unsigned short));
    
    UploadMesh(&mesh, true);
    
    return mesh;
}

//--------------------------------------

float orbit_camera_update_azimuth(
    const float azimuth, 
    const float mouse_dx,
    const float dt)
{
    return azimuth + 1.0f * dt * -mouse_dx;
}

float orbit_camera_update_altitude(
    const float altitude, 
    const float mouse_dy,
    const float dt)
{
    return clampf(altitude + 1.0f * dt * mouse_dy, 0.0, 0.4f * PIf);
}

float orbit_camera_update_distance(
    const float distance, 
    const float dt)
{
    return clampf(distance +  20.0f * dt * -GetMouseWheelMove(), 0.1f, 100.0f);
}

void orbit_camera_update(
    Camera3D& cam, 
    float& camera_azimuth,
    float& camera_altitude,
    float& camera_distance,
    const vec3 target,
    const float mouse_dx,
    const float mouse_dy,
    const float dt)
{
    camera_azimuth = orbit_camera_update_azimuth(camera_azimuth, mouse_dx, dt);
    camera_altitude = orbit_camera_update_altitude(camera_altitude, mouse_dy, dt);
    camera_distance = orbit_camera_update_distance(camera_distance, dt);
    
    quat rotation_azimuth = quat_from_angle_axis(camera_azimuth, vec3(0, 1, 0));
    vec3 position = quat_mul_vec3(rotation_azimuth, vec3(0, 0, camera_distance));
    vec3 axis = normalize(cross(position, vec3(0, 1, 0)));
    
    quat rotation_altitude = quat_from_angle_axis(camera_altitude, axis);
    
    vec3 eye = target + quat_mul_vec3(rotation_altitude, position);

    cam.target = (Vector3){ target.x, target.y, target.z };
    cam.position = (Vector3){ eye.x, eye.y, eye.z };
}

//--------------------------------------

void draw_axis(const vec3 pos, const quat rot, const float scale = 1.0f)
{
    vec3 axis0 = pos + quat_mul_vec3(rot, scale * vec3(1.0f, 0.0f, 0.0f));
    vec3 axis1 = pos + quat_mul_vec3(rot, scale * vec3(0.0f, 1.0f, 0.0f));
    vec3 axis2 = pos + quat_mul_vec3(rot, scale * vec3(0.0f, 0.0f, 1.0f));
    
    DrawLine3D(to_Vector3(pos), to_Vector3(axis0), RED);
    DrawLine3D(to_Vector3(pos), to_Vector3(axis1), GREEN);
    DrawLine3D(to_Vector3(pos), to_Vector3(axis2), BLUE);
}

//--------------------------------------

struct kform
{
    vec3 pos = vec3(0, 0, 0);     // Position
    quat rot = quat(1, 0, 0, 0);  // Rotation
    vec3 scl = vec3(1, 1, 1);     // Scale
    vec3 vel = vec3(0, 0, 0);     // Linear Velocity
    vec3 ang = vec3(0, 0, 0);     // Angular Velocity
    vec3 svl = vec3(0, 0, 0);     // Scalar Velocity
};

kform kform_mul(kform v, kform w)
{
    kform out;
    out.pos = quat_mul_vec3(v.rot, w.pos * v.scl) + v.pos;
    
    out.rot = quat_mul(v.rot, w.rot);
    
    out.scl = w.scl * v.scl;
    
    out.vel = quat_mul_vec3(v.rot, w.vel * v.scl) + v.vel + 
        cross(v.ang, quat_mul_vec3(v.rot, w.pos * v.scl)) +
        quat_mul_vec3(v.rot, w.pos * v.scl * v.svl);
    
    out.ang = quat_mul_vec3(v.rot, w.ang) + v.ang;
    
    out.svl = w.svl + v.svl;
    
    return out;
}

kform kform_div(kform v, kform w)
{
    kform out;
    out.pos = quat_inv_mul_vec3(v.rot, w.pos - v.pos);
    
    out.rot = quat_inv_mul(v.rot, w.rot);
    
    out.scl = w.scl / v.scl;
    
    out.vel = quat_inv_mul_vec3(v.rot, w.vel - v.vel - 
        cross(v.ang, quat_mul_vec3(v.rot, out.pos * v.scl))) -
        quat_mul_vec3(v.rot, out.pos * v.scl * v.svl);
    
    out.ang = quat_inv_mul_vec3(v.rot, w.ang - v.ang);
    
    out.svl = w.svl - v.svl;

    return out;
}

kform kform_inv(kform v)
{
    kform out;
    out.pos = quat_inv_mul_vec3(v.rot, -v.pos);
    
    out.rot = quat_inv(v.rot);
    
    out.scl = 1.0f / v.scl;
    
    out.vel = quat_inv_mul_vec3(v.rot, -v.vel - 
        cross(v.ang, quat_mul_vec3(v.rot, out.pos * v.scl))) -
        quat_mul_vec3(v.rot, out.pos * v.scl * v.svl);
    
    out.ang = quat_inv_mul_vec3(v.rot, -v.ang);
    
    out.svl = -v.svl;
    return out;
}

kform kform_inv_mul(kform v, kform w)
{
    return kform_mul(kform_inv(v), w);
}

kform kform_mul_inv(kform v, kform w)
{
    return kform_mul(v, kform_inv(w));
}

vec3 kform_mul_dir(kform v, vec3 d)
{
    return quat_mul_vec3(v.rot, d);
}

vec3 kform_mul_pos(kform v, vec3 pos)
{
    return quat_mul_vec3(v.rot, pos * v.scl) + v.pos;
}

vec3 kform_mul_vel(kform v, vec3 pos, vec3 vel)
{
    return quat_mul_vec3(v.rot, vel * v.scl) + v.vel + 
        cross(v.ang, quat_mul_vec3(v.rot, pos * v.scl)) +
        quat_mul_vec3(v.rot, pos * v.scl * v.svl);
}

kform kform_from_pos_vel(vec3 pos, vec3 vel)
{
    kform out;
    out.pos = pos;
    out.vel = vel;
    return out;
}

kform kform_rot_ang_between(kform dir0, kform dir1)
{
    quat rot = quat_between(dir0.pos, dir1.pos);
    quat drot = quat_between_dx(dir0.pos, dir0.vel, dir1.pos, dir1.vel);
    
    kform out;
    out.rot = rot;
    out.ang = quat_delta_to_angular_velocity(rot, drot);
    
    return out;
}

kform kform_pos_vel_normalize(kform d)
{
    kform out = d;
    out.pos = normalize(d.pos);
    out.vel = normalize_dx(d.pos, d.vel);
    return out;
}

kform kform_rot_ang_only(kform d)
{
    d.pos = vec3();
    d.scl = vec3(1,1,1);
    d.vel = vec3();
    d.svl = vec3();
    return d;
}

kform kform_rot_only(kform d)
{
    d.pos = vec3();
    d.scl = vec3(1,1,1);
    d.vel = vec3();
    d.ang = vec3();
    d.svl = vec3();
    return d;
}

kform kform_pos_vel_only(kform d)
{
    d.rot = quat();
    d.scl = vec3(1,1,1);
    d.ang = vec3();
    d.svl = vec3();
    return d;
}

kform kform_pos_only(kform d)
{
    d.rot = quat();
    d.scl = vec3(1,1,1);
    d.vel = vec3();
    d.ang = vec3();
    d.svl = vec3();
    return d;
}

//--------------------------------------

struct pose
{
    pose(int nbones)
    {
        pos.resize(nbones);
        rot.resize(nbones);
        scl.resize(nbones);
        vel.resize(nbones);
        ang.resize(nbones);
        svl.resize(nbones);
    }
    
    array1d<vec3> pos; // Positions
    array1d<quat> rot; // Rotations
    array1d<vec3> scl; // Scales
    array1d<vec3> vel; // Linear Velocities
    array1d<vec3> ang; // Angular Velocities
    array1d<vec3> svl; // Scalar Velocities
    
    inline kform operator()(int i) const
    {
        assert(i >= 0 && i < pos.size);
        
        kform out;
        out.pos = pos(i);
        out.rot = rot(i);
        out.scl = scl(i);
        out.vel = vel(i);
        out.ang = ang(i);
        out.svl = svl(i);
        return out;
    }
};

//--------------------------------------

void op_sample_linear(
    pose& output,                 // Output Pose
    const slice2d<vec3> anim_pos, // Animation Positions
    const slice2d<quat> anim_rot, // Animation Rotations
    const slice2d<vec3> anim_scl, // Animation Scales
    const float time,             // Time
    const float dt)               // Delta Time between Frames
{
    int st = (int)(time / dt);
    int s0 = clamp(st + 0, 0, anim_pos.rows - 1); // Previous Frame Index
    int s1 = clamp(st + 1, 0, anim_pos.rows - 1); // Next Frame Index
    float alpha = fmod(time / dt, 1.0f);          // Interpolation Value
    
    for (int j = 0; j < output.pos.size; j++)
    {
        output.pos(j) = lerp(anim_pos(s0, j), anim_pos(s1, j), alpha);
        
        output.vel(j) = (anim_pos(s1, j) - anim_pos(s0, j)) / dt;
        
        output.rot(j) = quat_slerp_shortest(
            anim_rot(s0, j), anim_rot(s1, j), alpha);
            
        output.ang(j) = quat_to_scaled_angle_axis(
            quat_abs(quat_mul_inv(anim_rot(s1, j), anim_rot(s0, j)))) / dt;
        
        output.scl(j) = eerp(anim_scl(s0, j), anim_scl(s1, j), alpha);
        
        output.svl(j) = log(anim_scl(s1, j) / anim_scl(s0, j)) / dt;
    }
}

//--------------------------------------

static inline void hermite(
    vec3& pos,
    vec3& vel, 
    float x, 
    vec3 p0,
    vec3 p1, 
    vec3 v0,
    vec3 v1,
    float dt)
{
    float w0 = 2*x*x*x - 3*x*x + 1;
    float w1 = 3*x*x - 2*x*x*x;
    float w2 = x*x*x - 2*x*x + x;
    float w3 = x*x*x - x*x;
    
    float q0 = 6*x*x - 6*x;
    float q1 = 6*x - 6*x*x;
    float q2 = 3*x*x - 4*x + 1;
    float q3 = 3*x*x - 2*x;
    
    pos = w0*p0 + w1*p1 + w2*v0 + w3*v1;
    vel = (q0*p0 + q1*p1 + q2*v0 + q3*v1) / dt;
}

static inline void catmull_rom(
    vec3& pos,
    vec3& vel,
    float x,
    vec3 p0,
    vec3 p1, 
    vec3 p2,
    vec3 p3,
    float dt)
{
    vec3 v1 = ((p1 - p0) + (p2 - p1)) / 2;
    vec3 v2 = ((p2 - p1) + (p3 - p2)) / 2;
    return hermite(pos, vel, x, p1, p2, v1, v2, dt);
}

static inline void quat_hermite(
    quat& rot,
    vec3& vel, 
    float x, 
    quat r0,
    quat r1, 
    vec3 v0,
    vec3 v1,
    float dt)
{
    float w1 = 3*x*x - 2*x*x*x;
    float w2 = x*x*x - 2*x*x + x;
    float w3 = x*x*x - x*x;
    
    float q1 = 6*x - 6*x*x;
    float q2 = 3*x*x - 4*x + 1;
    float q3 = 3*x*x - 2*x;
    
    vec3 r1_sub_r0 = quat_to_scaled_angle_axis(quat_abs(quat_mul_inv(r1, r0)));   
    
    rot = quat_mul(quat_from_scaled_angle_axis(w1*r1_sub_r0 + w2*v0 + w3*v1), r0);
    vel = (q1*r1_sub_r0 + q2*v0 + q3*v1) / dt;
}

static inline void quat_catmull_rom(
    quat& rot,
    vec3& vel,
    float x,
    quat r0,
    quat r1, 
    quat r2,
    quat r3,
    float dt)
{
    vec3 r1_sub_r0 = quat_to_scaled_angle_axis(quat_abs(quat_mul_inv(r1, r0)));
    vec3 r2_sub_r1 = quat_to_scaled_angle_axis(quat_abs(quat_mul_inv(r2, r1)));
    vec3 r3_sub_r2 = quat_to_scaled_angle_axis(quat_abs(quat_mul_inv(r3, r2)));
  
    vec3 v1 = (r1_sub_r0 + r2_sub_r1) / 2;
    vec3 v2 = (r2_sub_r1 + r3_sub_r2) / 2;
    return quat_hermite(rot, vel, x, r1, r2, v1, v2, dt);
}

static inline void scale_hermite(
    vec3& scl,
    vec3& svl, 
    float x, 
    vec3 s0,
    vec3 s1, 
    vec3 v0,
    vec3 v1,
    float dt)
{
    float w1 = 3*x*x - 2*x*x*x;
    float w2 = x*x*x - 2*x*x + x;
    float w3 = x*x*x - x*x;
    
    float q1 = 6*x - 6*x*x;
    float q2 = 3*x*x - 4*x + 1;
    float q3 = 3*x*x - 2*x;
    
    vec3 s1_sub_s0 = log(s1 / s0);   
    
    scl = exp(w1*s1_sub_s0 + w2*v0 + w3*v1) * s0;
    svl = (q1*s1_sub_s0 + q2*v0 + q3*v1) / dt;
}

static inline void scale_catmull_rom(
    vec3& scl,
    vec3& svl,
    float x,
    vec3 s0,
    vec3 s1, 
    vec3 s2,
    vec3 s3,
    float dt)
{
    vec3 s1_sub_s0 = log(s1 / s0);
    vec3 s2_sub_s1 = log(s2 / s1);
    vec3 s3_sub_s2 = log(s3 / s2);
  
    vec3 v1 = (s1_sub_s0 + s2_sub_s1) / 2;
    vec3 v2 = (s2_sub_s1 + s3_sub_s2) / 2;
    return scale_hermite(scl, svl, x, s1, s2, v1, v2, dt);
}

void op_sample_cubic(
    pose& output,
    const slice2d<vec3> anim_pos,
    const slice2d<quat> anim_rot,
    const slice2d<vec3> anim_scl,
    const float time,
    const float dt)
{
    int st = (int)(time / dt);
    int s0 = clamp(st - 1, 0, anim_pos.rows - 1); 
    int s1 = clamp(st - 0, 0, anim_pos.rows - 1); 
    int s2 = clamp(st + 1, 0, anim_pos.rows - 1); 
    int s3 = clamp(st + 2, 0, anim_pos.rows - 1);
    float alpha = fmod(time / dt, 1.0f);
    
    for (int j = 0; j < anim_pos.cols; j++)
    {
        catmull_rom(
            output.pos(j),
            output.vel(j),
            alpha,
            anim_pos(s0, j),
            anim_pos(s1, j),
            anim_pos(s2, j),
            anim_pos(s3, j),
            dt);
    }
    
    for (int j = 0; j < anim_rot.cols; j++)
    {
        quat_catmull_rom(
            output.rot(j),
            output.ang(j),
            alpha,
            anim_rot(s0, j),
            anim_rot(s1, j),
            anim_rot(s2, j),
            anim_rot(s3, j),
            dt);
    }
    
    for (int j = 0; j < anim_scl.cols; j++)
    {
        scale_catmull_rom(
            output.scl(j),
            output.svl(j),
            alpha,
            anim_scl(s0, j),
            anim_scl(s1, j),
            anim_scl(s2, j),
            anim_scl(s3, j),
            dt);
    }
}

//--------------------------------------

void op_timescale(
    pose& output,
    const float factor)
{
    for (int i = 0; i < output.pos.size; i++)
    {
        output.vel(i) = output.vel(i) * factor;      
        output.ang(i) = output.ang(i) * factor;      
        output.svl(i) = output.svl(i) * factor;      
    }
}

//--------------------------------------

void op_remove_global_root(pose& pose)
{
    vec3 root_pos = pose.pos(0);
    quat root_rot = pose.rot(0);

    pose.pos(0) = quat_inv_mul_vec3(root_rot, pose.pos(0) - root_pos);
    pose.vel(0) = quat_inv_mul_vec3(root_rot, pose.vel(0));
    pose.rot(0) = quat_inv_mul(root_rot, pose.rot(0));
    pose.ang(0) = quat_inv_mul_vec3(root_rot, pose.ang(0));
}

//--------------------------------------

void op_fk(
    pose& global,
    const pose& local,
    const slice1d<int> parents)
{
    for (int i = 0; i < parents.size; i++)
    {
        if (parents(i) == -1)
        {
            global.pos(i) = local.pos(i);
            global.rot(i) = local.rot(i);
            global.scl(i) = local.scl(i);
            global.vel(i) = local.vel(i);
            global.ang(i) = local.ang(i);
            global.svl(i) = local.svl(i);
        }
        else
        {
            // Ensure parent global transforms have already been computed
            assert(parents(i) < i);
            
            vec3 par_pos = global.pos(parents(i));
            quat par_rot = global.rot(parents(i));
            vec3 par_scl = global.scl(parents(i));
            vec3 par_vel = global.vel(parents(i));
            vec3 par_ang = global.ang(parents(i));
            vec3 par_svl = global.svl(parents(i));
            
            // Position
            global.pos(i) = quat_mul_vec3(par_rot, local.pos(i) * par_scl) + 
                par_pos;
            
            // Rotation
            global.rot(i) = quat_mul(par_rot, local.rot(i));
            
            // Scale
            global.scl(i) = local.scl(i) * par_scl;
            
            // Linear Velocity
            global.vel(i) = quat_mul_vec3(par_rot, local.vel(i) * par_scl) + 
                par_vel + 
                cross(par_ang, quat_mul_vec3(par_rot, local.pos(i) * par_scl)) +
                quat_mul_vec3(par_rot, local.pos(i) * par_scl * par_svl);
            
            // Angular Velocity
            global.ang(i) = quat_mul_vec3(par_rot, local.ang(i)) + par_ang;
            
            // Scalar Velocity
            global.svl(i) = local.svl(i) + par_svl;
        }
    }
}

void op_fk_static(
    pose& global,
    const pose& local,
    const slice1d<int> parents)
{
    global.vel.zero();
    global.ang.zero();
    global.svl.zero();
    
    for (int i = 0; i < parents.size; i++)
    {
        if (parents(i) == -1)
        {
            global.pos(i) = local.pos(i);
            global.rot(i) = local.rot(i);
            global.scl(i) = local.scl(i);
        }
        else
        {
            // Ensure parent global transforms have already been computed.
            assert(parents(i) < i);
            
            vec3 par_pos = global.pos(parents(i));
            quat par_rot = global.rot(parents(i));
            vec3 par_scl = global.scl(parents(i));
            
            global.pos(i) = quat_mul_vec3(par_rot, local.pos(i) * par_scl) + par_pos;
            global.rot(i) = quat_mul(par_rot, local.rot(i));
            global.scl(i) = local.scl(i) * par_scl;
        }
    }
}

//--------------------------------------

void op_bk(
    pose& local,
    const pose& global,
    const slice1d<int> parents)
{
    for (int i = 0; i < parents.size; i++)
    {
        if (parents(i) == -1)
        {
            local.pos(i) = global.pos(i);
            local.rot(i) = global.rot(i);
            local.scl(i) = global.scl(i);
            local.vel(i) = global.vel(i);
            local.ang(i) = global.ang(i);
            local.svl(i) = global.svl(i);
        }
        else
        {
            vec3 par_pos = global.pos(parents(i));
            quat par_rot = global.rot(parents(i));
            vec3 par_scl = global.scl(parents(i));
            vec3 par_vel = global.vel(parents(i));
            vec3 par_ang = global.ang(parents(i));
            vec3 par_svl = global.svl(parents(i));
            
            // Position
            local.pos(i) = quat_inv_mul_vec3(par_rot, global.pos(i) - par_pos);
            
            // Rotation
            local.rot(i) = quat_inv_mul(par_rot, global.rot(i));
            
            // Scale
            local.scl(i) = global.scl(i) / par_scl;
            
            // Linear Velocity
            local.vel(i) = quat_inv_mul_vec3(par_rot, global.vel(i) - par_vel - 
                cross(par_ang, quat_mul_vec3(par_rot, local.pos(i) * par_scl))) -
                quat_mul_vec3(par_rot, local.pos(i) * par_scl * par_svl);
            
            // Angular Velocity
            local.ang(i) = quat_inv_mul_vec3(par_rot, global.ang(i) - par_ang);
            
            // Scalar Velocity
            local.svl(i) = global.svl(i) - par_svl;
        }
    }
}

//--------------------------------------

void op_finite_diff(
    pose& output,
    const pose& input_curr,
    const pose& input_prev,
    const float dt)
{
    for (int i = 0; i < output.pos.size; i++)
    {
        output.vel(i) = (input_curr.pos(i) - input_prev.pos(i)) / dt;      
        
        output.ang(i) = quat_to_scaled_angle_axis(quat_abs(
            quat_mul_inv(input_curr.rot(i), input_prev.rot(i)))) / dt;
            
        output.svl(i) = log(input_curr.scl(i) / input_prev.scl(i)) / dt;      
    }
}

void op_finite_diff_inplace(
    pose& output,
    const pose& input_prev,
    const float dt)
{
    for (int i = 0; i < output.pos.size; i++)
    {
        output.vel(i) = (output.pos(i) - input_prev.pos(i)) / dt;      
        
        output.ang(i) = quat_to_scaled_angle_axis(quat_abs(
            quat_mul_inv(output.rot(i), input_prev.rot(i)))) / dt;
            
        output.svl(i) = log(output.scl(i) / input_prev.scl(i)) / dt;      
    }
}

//--------------------------------------

void op_blend_static(
    pose& output,
    const pose& lhs,
    const pose& rhs,
    const float alpha)
{
    for (int i = 0; i < output.pos.size; i++)
    {
        output.pos(i) = lerp(lhs.pos(i), rhs.pos(i), alpha);
        output.rot(i) = quat_slerp_shortest(lhs.rot(i), rhs.rot(i), alpha);
        output.scl(i) = eerp(lhs.scl(i), rhs.scl(i), alpha);
        output.vel(i) = lerp(lhs.vel(i), rhs.vel(i), alpha);
        output.ang(i) = lerp(lhs.ang(i), rhs.ang(i), alpha);
        output.svl(i) = lerp(lhs.svl(i), rhs.svl(i), alpha);
    }
}

//--------------------------------------

float smoothstep(float x)
{
    return 3*x*x - 2*x*x*x;
}

float smoothstep_dx(float x)
{
    return 6*x - 6*x*x;
}

float smoothstep_blend_alpha(
    float blend_time,
    float blend_duration,
    float eps = 1e-8f)
{
    float x = clampf(blend_time / maxf(blend_duration, eps), 0.0f, 1.0f);    
    return smoothstep(x);
}

float smoothstep_blend_alpha_dx(
    float blend_time,
    float blend_duration,
    float blend_dt,
    float eps = 1e-8f)
{
    float x = clampf(blend_time / maxf(blend_duration, eps), 0.0f, 1.0f);    
    return smoothstep_dx(x) * (blend_dt / maxf(blend_duration, eps));
}

//--------------------------------------

void op_blend(
    pose& output,
    const pose& lhs,
    const pose& rhs,
    const float alpha,
    const float alpha_vel,
    const float dt)
{
    for (int i = 0; i < output.pos.size; i++)
    {
        output.pos(i) = lerp(lhs.pos(i), rhs.pos(i), alpha);
        output.rot(i) = quat_slerp_shortest(lhs.rot(i), rhs.rot(i), alpha);
        output.scl(i) = eerp(lhs.scl(i), rhs.scl(i), alpha);
        
        output.vel(i) = lerp(lhs.vel(i), rhs.vel(i), alpha) + 
            (rhs.pos(i) - lhs.pos(i)) * (alpha_vel / dt);
        
        output.ang(i) = lerp(lhs.ang(i), rhs.ang(i), alpha) +
            quat_to_scaled_angle_axis(quat_abs(
                quat_mul_inv(rhs.rot(i), lhs.rot(i)))) * (alpha_vel / dt);
            
        output.svl(i) = lerp(lhs.svl(i), rhs.svl(i), alpha) +
            log(rhs.scl(i) / lhs.scl(i)) * (alpha_vel / dt);
    }
}

//--------------------------------------

void op_additive_inplace_static(
    pose& output,       // Pose to apply the additive to
    const pose& add,    // Additive pose
    const float alpha,  // Weight for additive
    const float dt)
{
    for (int i = 0; i < output.pos.size; i++)
    {
        quat add_rot = quat_from_scaled_angle_axis(
            alpha * quat_to_scaled_angle_axis(quat_abs(add.rot(i))));
        
        output.pos(i) = alpha * add.pos(i) + output.pos(i);
        
        output.rot(i) = quat_mul(add_rot, output.rot(i));
        
        output.scl(i) = exp(alpha * log(add.scl(i))) * output.scl(i);
        
        output.vel(i) = output.vel(i) + alpha * add.vel(i);
        
        output.ang(i) = quat_mul_vec3(add_rot, output.ang(i)) + 
            alpha * add.ang(i);
        
        output.svl(i) = output.svl(i) + alpha * add.svl(i);
    }
}

void op_additive_local_inplace_static(
    pose& output,       // Pose to apply the additive to
    const pose& add,    // Additive pose
    const float alpha)  // Weight for additive
{
    for (int i = 0; i < output.pos.size; i++)
    {
        quat add_rot = quat_from_scaled_angle_axis(
            alpha * quat_to_scaled_angle_axis(quat_abs(add.rot(i))));
        
        quat cur_rot = output.rot(i);

        output.pos(i) = alpha * add.pos(i) + output.pos(i);
        
        output.rot(i) = quat_mul(cur_rot, add_rot);
        
        output.scl(i) = exp(alpha * log(add.scl(i))) * output.scl(i);
        
        output.vel(i) = output.vel(i) + alpha * add.vel(i);
        
        output.ang(i) = output.ang(i) + 
            quat_mul_vec3(cur_rot, alpha * add.ang(i));
        
        output.svl(i) = output.svl(i) + alpha * add.svl(i);
    }
}

void op_additive_inplace(
    pose& output,          // Pose to apply the additive to
    const pose& add,       // Additive pose
    const float alpha,     // Weight for additive
    const float alpha_vel, // Rate of change for the weight
    const float dt)        // Delta time
{
    for (int i = 0; i < output.pos.size; i++)
    {
        quat add_rot = quat_from_scaled_angle_axis(
            alpha * quat_to_scaled_angle_axis(quat_abs(add.rot(i))));
        
        output.pos(i) = alpha * add.pos(i) + output.pos(i);
        
        output.rot(i) = quat_mul(add_rot, output.rot(i));
        
        output.scl(i) = exp(alpha * log(add.scl(i))) * output.scl(i);

        output.vel(i) = output.vel(i) + alpha * add.vel(i) + 
            add.pos(i) * (alpha_vel / dt);
        
        output.ang(i) = quat_mul_vec3(add_rot, output.ang(i)) + 
            alpha * add.ang(i) +
            quat_to_scaled_angle_axis(quat_abs(add.rot(i))) * (alpha_vel / dt);
            
        output.svl(i) = output.svl(i) + alpha * add.svl(i) +
            log(add.scl(i)) * (alpha_vel / dt);
    }
}

void op_additive_local_inplace(
    pose& output,          // Pose to apply the additive to
    const pose& add,       // Additive pose
    const float alpha,     // Weight for additive
    const float alpha_vel, // Rate of change for the weight
    const float dt)        // Delta time
{
    for (int i = 0; i < output.pos.size; i++)
    {
        quat add_rot = quat_from_scaled_angle_axis(
            alpha * quat_to_scaled_angle_axis(quat_abs(add.rot(i))));
        
        quat cur_rot = output.rot(i);
        
        output.pos(i) = output.pos(i) + alpha * add.pos(i);
        
        output.rot(i) = quat_mul(cur_rot, add_rot);
        
        output.scl(i) = output.scl(i) * exp(alpha * log(add.scl(i)));

        output.vel(i) = output.vel(i) + alpha * add.vel(i) + 
            add.pos(i) * (alpha_vel / dt);
        
        output.ang(i) = output.ang(i) + quat_mul_vec3(cur_rot, alpha * add.ang(i)) +
            quat_to_scaled_angle_axis(quat_abs(add.rot(i))) * (alpha_vel / dt);
            
        output.svl(i) = output.svl(i) + alpha * add.svl(i) +
            log(add.scl(i)) * (alpha_vel / dt);
    }
}

//--------------------------------------

void op_inertialize_offset(
    pose& offset,
    const pose& lhs,
    const pose& rhs)
{
    for (int i = 0; i < offset.pos.size; i++)
    {
        offset.pos(i) = lhs.pos(i) - rhs.pos(i);
        offset.rot(i) = quat_mul_inv(lhs.rot(i), rhs.rot(i));
        offset.scl(i) = lhs.scl(i) / rhs.scl(i);
        offset.vel(i) = lhs.vel(i) - rhs.vel(i);
        offset.ang(i) = lhs.ang(i) - rhs.ang(i);
        offset.svl(i) = lhs.svl(i) - rhs.svl(i);
    }
}

void op_inertialize(
    pose& output,               // Output Animation
    const pose& offset,         // Additive Offset
    const pose& input,          // Input Animation
    const float blend_time,     // Current blend time
    const float blend_duration, // Total blend duration
    const float eps = 1e-8f)
{
    // Scale and clamp blend time to be between 0 and 1
    float t = clampf(blend_time / (blend_duration + eps), 0.0f, 1.0f);
    
    // Compute coefficients determined by cubic decay
    float w0 = 2.0f*t*t*t - 3.0f*t*t + 1.0f;
    float w1 = (t*t*t - 2.0f*t*t + t) * blend_duration;
    float w2 = (6.0f*t*t - 6.0f*t) / (blend_duration + eps);
    float w3 = 3.0f*t*t - 4.0f*t + 1.0f;
  
    for (int i = 0; i < output.pos.size; i++)
    {
        // Compute decayed offset
        vec3 off_pos = w0 * offset.pos(i) + w1 * offset.vel(i);
        vec3 off_vel = w2 * offset.pos(i) + w3 * offset.vel(i);
        quat off_rot = quat_from_scaled_angle_axis(
            w0 * quat_to_scaled_angle_axis(offset.rot(i)) + w1 * offset.ang(i));
        
        vec3 off_ang = w2 * quat_to_scaled_angle_axis(offset.rot(i)) + 
            w3 * offset.ang(i);
        vec3 off_scl = exp(w0 * log(offset.scl(i)) + w1 * offset.svl(i));
        vec3 off_svl = w2 * log(offset.scl(i)) + w3 * offset.svl(i);
        
        // Apply decayed offset additively
        output.pos(i) = input.pos(i) + off_pos;
        output.vel(i) = input.vel(i) + off_vel;        
        output.rot(i) = quat_mul(off_rot, input.rot(i));
        output.ang(i) = quat_mul_vec3(off_rot, input.ang(i)) + off_ang;
        output.scl(i) = input.scl(i) * off_scl;
        output.svl(i) = input.svl(i) + off_svl;
    }
}

//--------------------------------------

void op_extrapolate_decayed(
    pose& output,
    const pose& input,
    const float dt,
    const float halflife,
    const float eps = 1e-8f)
{
    // Compute amount of extrapolation and velocity decay factor
    float y = 0.69314718056f / (halflife + eps);
    float alpha = (1.0f - fast_negexpf(y * dt)) / (y + eps);
    float dalpha = fast_negexpf(y * dt);

    for (int i = 0; i < input.pos.size; i++)
    {   
        if (i == 0)
        {
            // Don't extrapolate root
            output.pos(i) = input.pos(i);
            output.rot(i) = input.rot(i);
            output.scl(i) = input.scl(i);
            output.vel(i) = input.vel(i);
            output.ang(i) = input.ang(i);
            output.svl(i) = input.svl(i);
        }
        else
        {
            // Extrapolated state
            output.pos(i) = input.pos(i) + input.vel(i) * alpha;
            output.rot(i) = quat_mul(quat_from_scaled_angle_axis(
                input.ang(i) * alpha), input.rot(i));
            output.scl(i) = input.scl(i) * exp(input.svl(i) * alpha);
            
            // Decay velocities
            output.vel(i) = input.vel(i) * dalpha;
            output.ang(i) = input.ang(i) * dalpha;
            output.svl(i) = input.svl(i) * dalpha;
        }
    }
}

void op_dead_blending(
    pose& output,
    pose& extrapolated_pose,
    const pose& transition_pose,
    const pose& input,
    const float extrapolation_halflife,
    const float blend_time,
    const float blend_duration,
    const float dt)
{
    // Extrapolate transition pose forward in time
    op_extrapolate_decayed(
        extrapolated_pose,
        transition_pose,
        blend_time,
        extrapolation_halflife);
    
    // Compute blend factor
    float blend_alpha = smoothstep_blend_alpha(
        blend_time, blend_duration);
        
    // Compute blend factor velocity
    float blend_alpha_vel = smoothstep_blend_alpha_dx(
        blend_time, blend_duration, dt);
    
    // Apply Blend
    op_blend(
        output,
        extrapolated_pose,
        input,
        blend_alpha,
        blend_alpha_vel,
        dt);
}

//--------------------------------------

void op_lookat_static(
    pose& local,
    const pose& global,
    const vec3 target_pos,
    const vec3 fwd = vec3(0,1,0))
{
    // Find the target look-at position local to the head
    vec3 tar_pos = quat_inv_mul_vec3(global.rot(Bone_Head), 
        target_pos - global.pos(Bone_Head));
    
    // Find the rotation between the target and the forward dir
    quat diff = quat_between(fwd, normalize(tar_pos));
    
    // Rotate the head to face toward the target
    local.rot(Bone_Head) = quat_mul(local.rot(Bone_Head), diff);
}

void op_lookat(
    pose& local,
    const pose& global,
    const vec3 target_pos,
    const vec3 target_vel,
    const vec3 fwd = vec3(0,1,0))
{
    // Find the target look-at position local to the head
    kform tar_pos = kform_inv_mul(global(Bone_Head), 
        kform_from_pos_vel(target_pos, target_vel));

    // Find the rotation between the target and the forward dir
    kform diff = kform_rot_ang_between(
        kform_from_pos_vel(fwd, vec3()), 
        kform_pos_vel_normalize(tar_pos));

    // Rotate the head to face toward the target
    kform head_rot = kform_mul(local(Bone_Head), diff);
    local.rot(Bone_Head) = head_rot.rot;
    local.ang(Bone_Head) = head_rot.ang;
}

//--------------------------------------

void op_two_bone_ik_static(
    pose& local,
    const pose& global,
    const vec3 heel_target,
    const vec3 fwd = vec3(0.0f, 1.0f, 0.0f),
    const float max_length_buffer = 0.01f)
{
    float max_extension = 
        length(global.pos(Bone_LeftUpLeg) - global.pos(Bone_LeftLeg)) + 
        length(global.pos(Bone_LeftLeg) - global.pos(Bone_LeftFoot)) - 
        max_length_buffer;
    
    vec3 target_clamp = heel_target;
    if (length(heel_target - global.pos(Bone_LeftUpLeg)) > max_extension)
    {
        target_clamp = global.pos(Bone_LeftUpLeg) + max_extension * normalize(heel_target - global.pos(Bone_LeftUpLeg));
    }
    
    vec3 axis_dwn = normalize(global.pos(Bone_LeftFoot) - global.pos(Bone_LeftUpLeg));
    vec3 axis_rot = normalize(cross(axis_dwn, quat_mul_vec3(global.rot(Bone_RightLeg), fwd)));

    vec3 a = global.pos(Bone_LeftUpLeg);
    vec3 b = global.pos(Bone_LeftLeg);
    vec3 c = global.pos(Bone_LeftFoot);
    vec3 t = target_clamp;
    
    float lab = length(b - a);
    float lcb = length(b - c);
    float lat = length(t - a);

    float ac_ab_0 = acosf(clampf(dot(normalize(c - a), normalize(b - a)), -1.0f, 1.0f));
    float ba_bc_0 = acosf(clampf(dot(normalize(a - b), normalize(c - b)), -1.0f, 1.0f));

    float ac_ab_1 = acosf(clampf((lab * lab + lat * lat - lcb * lcb) / (2.0f * lab * lat), -1.0f, 1.0f));
    float ba_bc_1 = acosf(clampf((lab * lab + lcb * lcb - lat * lat) / (2.0f * lab * lcb), -1.0f, 1.0f));

    quat r0 = quat_from_angle_axis(ac_ab_1 - ac_ab_0, axis_rot);
    quat r1 = quat_from_angle_axis(ba_bc_1 - ba_bc_0, axis_rot);

    vec3 c_a = normalize(global.pos(Bone_LeftFoot) - global.pos(Bone_LeftUpLeg));
    vec3 t_a = normalize(target_clamp - global.pos(Bone_LeftUpLeg));

    quat r2 = quat_from_angle_axis(
        acosf(clampf(dot(c_a, t_a), -1.0f, 1.0f)),
        normalize(cross(c_a, t_a)));
    
    local.rot(Bone_LeftUpLeg) = quat_inv_mul(global.rot(Bone_Hips), quat_mul(r2, quat_mul(r0, global.rot(Bone_LeftUpLeg))));
    local.rot(Bone_LeftLeg) = quat_inv_mul(global.rot(Bone_LeftUpLeg), quat_mul(r1, global.rot(Bone_LeftLeg)));
}

void op_extrapolate_prev(
    pose& proj,
    const pose& pose,
    const float dt)
{
    for (int i = 0; i < pose.pos.size; i++)
    {
        proj.pos(i) = pose.pos(i) - pose.vel(i) * dt;
        proj.rot(i) = quat_inv_mul(quat_from_scaled_angle_axis(pose.ang(i) * dt), pose.rot(i));
        proj.scl(i) = pose.scl(i) / exp(pose.svl(i) * dt);
        proj.vel(i) = pose.vel(i);
        proj.ang(i) = pose.ang(i);
        proj.svl(i) = pose.svl(i);
    }
}

void op_two_bone_ik(
    pose& local_curr,
    pose& local_temp,
    pose& global_curr,
    pose& global_temp,
    const slice1d<int> bone_parents,
    const vec3 heel_pos,
    const vec3 heel_vel,
    const float delta = 0.001f)
{
    op_extrapolate_prev(local_temp, local_curr, delta);
    
    op_fk_static(global_curr, local_curr, bone_parents);
    op_fk_static(global_temp, local_temp, bone_parents);
    
    op_two_bone_ik_static(local_curr, global_curr, heel_pos);
    op_two_bone_ik_static(local_temp, global_temp, heel_pos - delta * heel_vel);
    
    op_finite_diff_inplace(local_curr, local_temp, delta);
}

//--------------------------------------

void update_callback(void* args)
{
    ((std::function<void()>*)args)->operator()();
}

int main(void)
{
    // Init Window
    
    //const int screen_width = 1280;
    //const int screen_height = 720;
    const int screen_width = 640;
    const int screen_height = 480;
    
    SetConfigFlags(FLAG_VSYNC_HINT);
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(screen_width, screen_height, "raylib [propagating velocities]");
    SetTargetFPS(60);
    
    // Camera

    Camera3D camera = { 0 };
    camera.position = (Vector3){ 2.0f, 3.0f, 5.0f };
    camera.target = (Vector3){ -0.5f, 1.0f, 0.0f };
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;
    
    float camera_azimuth = 0.0f;
    float camera_altitude = 0.4f;
    float camera_distance = 3.0f;
    
    // Ground Plane
    
    Shader ground_plane_shader = LoadShader("./resources/checkerboard.vs", "./resources/checkerboard.fs");
    Mesh ground_plane_mesh = GenMeshPlane(40.0f, 40.0f, 10, 10);
    Model ground_plane_model = LoadModelFromMesh(ground_plane_mesh);
    ground_plane_model.materials[0].shader = ground_plane_shader;
    
    // Character
    
    character character_data;
    character_load(character_data, "./resources/character.bin");
    
    Shader character_shader = LoadShader("./resources/character.vs", "./resources/character.fs");
    Mesh character_mesh = make_character_mesh(character_data);
    Model character_model = LoadModelFromMesh(character_mesh);
    character_model.materials[0].shader = character_shader;
    
    // Load Animation Data
    
    database db;
    database_load(db, "./resources/database.bin");
    
    // Playback
  
    const float dt = 1.0f / 60.0f;
    float time0 = 0.0f;
    float time1 = 0.0f;
    float time_additive = 0.0f;
    //float playback = 0.1f;
    //float playback = 0.5f;
    float playback = 1.0f;
    //float playback = 0.0f;
    
    // Animations
    
    //int anim_index0 = 1;
    //int anim_index1 = 0;
    
    int anim_index0 = 2;
    int anim_index1 = 0;
    
    array2d<vec3> anim0_scl_data(db.range_stops(anim_index0) - db.range_starts(anim_index0), db.nbones());
    array2d<vec3> anim1_scl_data(db.range_stops(anim_index1) - db.range_starts(anim_index1), db.nbones());
    anim0_scl_data.set(vec3(1, 1, 1));
    anim1_scl_data.set(vec3(1, 1, 1));

    slice2d<vec3> anim0_pos = db.bone_positions.slice(db.range_starts(anim_index0), db.range_stops(anim_index0));
    slice2d<quat> anim0_rot = db.bone_rotations.slice(db.range_starts(anim_index0), db.range_stops(anim_index0));
    slice2d<vec3> anim0_scl = anim0_scl_data;
    slice2d<vec3> anim1_pos = db.bone_positions.slice(db.range_starts(anim_index1), db.range_stops(anim_index1));
    slice2d<quat> anim1_rot = db.bone_rotations.slice(db.range_starts(anim_index1), db.range_stops(anim_index1));
    slice2d<vec3> anim1_scl = anim1_scl_data;
    
    array2d<vec3> additive_pos_data(anim0_pos.rows, db.nbones());
    array2d<quat> additive_rot_data(anim0_pos.rows, db.nbones());
    array2d<vec3> additive_scl_data(anim0_pos.rows, db.nbones());
    
    additive_pos_data.zero();
    additive_rot_data.set(quat());
    additive_scl_data.set(vec3(1,1,1));
    
    for (int i = 0; i < additive_rot_data.rows; i++)
    {
        float angle0 = 0.5f * sinf(i * 0.1) + 0.25f;
        float angle1 = 0.5f * sinf(i * 0.123) + 0.25f;
        additive_rot_data(i, Bone_LeftArm) = quat_from_angle_axis(-angle0, vec3(0,1,0));
        additive_rot_data(i, Bone_RightArm) = quat_from_angle_axis(angle1, vec3(0,1,0));
    }
    
    slice2d<vec3> additive_pos = additive_pos_data;
    slice2d<quat> additive_rot = additive_rot_data;
    slice2d<vec3> additive_scl = additive_scl_data;
    
    // Pose Data
    
    pose local_anim0(db.nbones());
    pose local_anim1(db.nbones());
    pose local_final(db.nbones());
    pose local_root(db.nbones());
    pose local_additive(db.nbones());
    pose local_inertialization_dst(db.nbones());
    pose local_inertialization_off(db.nbones());
    pose local_dead_blending_transition(db.nbones());
    pose local_dead_blending_extrapolation(db.nbones());
    
    vec3 root_pos = vec3();
    vec3 root_vel = vec3();
    quat root_rot = quat();
    vec3 root_ang = vec3();
    
    pose local_curr(db.nbones());
    pose local_prev(db.nbones());
    pose local_diff(db.nbones());
    pose global_curr(db.nbones());
    pose global_prev(db.nbones());
    pose global_diff(db.nbones());
    pose local_back(db.nbones());

    // Sampling

    bool sample_cubic = true;
  
    // Blending
  
    //bool perform_blending = true;
    bool perform_blending = false;
    bool blend_positive = false;
    float blend_time = 0.0f;
    float blend_duration = 1.0f;
    float blend_change = 0.0f;
    float blend_alpha = 0.0f;
    float blend_alpha_vel = 0.0f;
    
    // Additive
    
    //bool perform_additive = true;
    bool perform_additive = false;
    bool additive_positive = true;
    bool additive_local = true;
    float additive_time = 0.0f;
    float additive_duration = 1.0f;
    float additive_change = 0.0f;
    float additive_alpha = 0.0f;
    float additive_alpha_vel = 0.0f;
    
    // Inertialization
    
    //bool perform_inertialization = true;
    bool perform_inertialization = false;
    bool inertialization_trigger = false;
    bool inertialization_active = false;
    bool inertialization_positive = true;
    float inertialization_time = 0.0f;
    float inertialization_duration = 0.25f;
  
    // Dead Blending
    
    //bool perform_dead_blending = true;
    bool perform_dead_blending = false;
    bool dead_blend_trigger = false;
    bool dead_blend_active = false;
    bool dead_blend_positive = true;
    float dead_blend_time = 0.0f;
    float dead_blend_duration = 0.5f;
    float dead_blending_extrapolation_halflife = 0.2f;
    
    // Look At
    
    //bool lookat_enabled = true;
    bool lookat_enabled = false;
    bool lookat_velocities = true;
    bool lookat_target_path = true;
    float lookat_target_path_time = 0.0f;
    float lookat_target_path_speed = 3.0f;
    vec3 lookat_target_curr = vec3();
    vec3 lookat_target_prev = vec3();
    vec3 lookat_target_vel = vec3();
    vec3 lookat_target_move = vec3();
    
    // IK
    
    //bool ik_enabled = true;
    bool ik_enabled = false;
    bool ik_velocities = true;
    bool ik_target_path = true;
    float ik_target_path_time = 0.0f;
    float ik_target_path_speed = 3.0f;
    vec3 ik_target_curr = vec3();
    vec3 ik_target_prev = vec3();
    vec3 ik_target_vel = vec3();
    vec3 ik_target_move = vec3();
    
    // Go
    
    auto update_func = [&]()
    {
        const float playback_dt = playback * dt;
      
        // Update Camera
        
        vec3 camera_target = global_curr.pos(0) + vec3(0, 0.75f, 0);
        
        orbit_camera_update(
            camera, 
            camera_azimuth,
            camera_altitude,
            camera_distance,
            camera_target,
            (IsKeyDown(KEY_LEFT_CONTROL) && IsMouseButtonDown(0)) ? GetMouseDelta().x : 0.0f,
            (IsKeyDown(KEY_LEFT_CONTROL) && IsMouseButtonDown(0)) ? GetMouseDelta().y : 0.0f,
            dt);
        
        // IK Target
        
        if (ik_target_path)
        {
            ik_target_prev = ik_target_curr;
            ik_target_curr = vec3(0.25 * sin(ik_target_path_speed * ik_target_path_time) + 0.25, 0.3 * cos(ik_target_path_speed * ik_target_path_time) + 0.4, 0.0f);
            ik_target_vel = (ik_target_curr - ik_target_prev) / dt;
            ik_target_path_time += dt;
        }
        else
        {
            vec3 ik_target_move_goal = vec3();
            if (!IsKeyDown(KEY_LEFT_CONTROL) && IsMouseButtonDown(1))
            {
                quat rotation_azimuth = quat_from_angle_axis(camera_azimuth, vec3(0, 1, 0));
                ik_target_move_goal = quat_mul_vec3(rotation_azimuth, 0.1f * vec3(GetMouseDelta().x, -GetMouseDelta().y, 0.0f));
            } 
        
            ik_target_move = damper_exact(ik_target_move, ik_target_move_goal, 0.1f, dt); 
            ik_target_prev = ik_target_curr;
            ik_target_curr = ik_target_curr + dt * ik_target_move;
            ik_target_vel = (ik_target_curr - ik_target_prev) / dt;
        }
        
        // Lookat Target
        
        if (lookat_target_path)
        {
            lookat_target_prev = lookat_target_curr;
            lookat_target_curr = vec3(0.75 * sin(lookat_target_path_speed * lookat_target_path_time) + 0.25, 1.0 * cos(lookat_target_path_speed * lookat_target_path_time) + 1.4, 1.0f);
            lookat_target_vel = (lookat_target_curr - lookat_target_prev) / dt;
            lookat_target_path_time += dt;
        }
        else
        {
            vec3 lookat_target_move_goal = vec3();
            if (!IsKeyDown(KEY_LEFT_CONTROL) && IsMouseButtonDown(1))
            {
                quat rotation_azimuth = quat_from_angle_axis(camera_azimuth, vec3(0, 1, 0));
                lookat_target_move_goal = quat_mul_vec3(rotation_azimuth, 0.25f * vec3(GetMouseDelta().x, -GetMouseDelta().y, 0.0f)); 
            }
            
            lookat_target_move = damper_exact(lookat_target_move, lookat_target_move_goal, 0.1f, dt); 
            lookat_target_prev = lookat_target_curr;
            lookat_target_curr = lookat_target_curr + dt * lookat_target_move;
            lookat_target_vel = (lookat_target_curr - lookat_target_prev) / dt;
        }
        
        // Tick
        
        time0 = fmod(time0 + playback_dt, (anim0_pos.rows - 1) * dt);            
        time1 = fmod(time1 + playback_dt, (anim1_pos.rows - 1) * dt);            
        time_additive = fmod(time_additive + playback_dt, (additive_pos.rows - 1) * dt);            
        
        // Sample Animations
        
        if (sample_cubic)
        {
            op_sample_cubic(
                local_anim0, 
                anim0_pos, 
                anim0_rot, 
                anim0_scl,
                time0, 
                dt);
            
            op_sample_cubic(
                local_anim1, 
                anim1_pos, 
                anim1_rot, 
                anim1_scl,
                time1, 
                dt);                
        }
        else
        {
            op_sample_linear(
                local_anim0, 
                anim0_pos, 
                anim0_rot, 
                anim0_scl,
                time0, 
                dt);
            
            op_sample_linear(
                local_anim1, 
                anim1_pos, 
                anim1_rot, 
                anim1_scl,
                time1, 
                dt);
        }
        
        op_timescale(local_anim0, playback_dt / dt);
        op_timescale(local_anim1, playback_dt / dt);
        op_remove_global_root(local_anim0);
        op_remove_global_root(local_anim1);

        // Blending
        
        if (perform_inertialization)
        {
            if (inertialization_trigger)
            {
                // Sample animation at previous frame to match current
                if (sample_cubic)
                {
                    op_sample_cubic(
                        local_inertialization_dst, 
                        inertialization_positive ? anim1_pos : anim0_pos, 
                        inertialization_positive ? anim1_rot : anim0_rot, 
                        inertialization_positive ? anim1_scl : anim0_scl,
                        (inertialization_positive ? time1 : time0) - playback_dt, 
                        dt);                
                }
                else
                {
                    op_sample_linear(
                        local_inertialization_dst, 
                        inertialization_positive ? anim1_pos : anim0_pos, 
                        inertialization_positive ? anim1_rot : anim0_rot, 
                        inertialization_positive ? anim1_scl : anim0_scl,
                        (inertialization_positive ? time1 : time0) - playback_dt, 
                        dt);
                }
                
                op_timescale(local_inertialization_dst, playback_dt / dt);
                op_remove_global_root(local_inertialization_dst);
                
                op_inertialize_offset(
                    local_inertialization_off,
                    local_final,
                    local_inertialization_dst);
                
                inertialization_active = true;
                inertialization_time = 0.0f;
                inertialization_trigger = false;
            }
            
            if (inertialization_active)
            {
                inertialization_time = clampf(inertialization_time + dt, 0.0f, inertialization_duration);
                
                op_inertialize(
                    local_final,
                    local_inertialization_off,
                    inertialization_positive ? local_anim1 : local_anim0,
                    inertialization_time,
                    inertialization_duration);
                    
                if (inertialization_time >= inertialization_duration)
                {
                    inertialization_active = false;
                }
            }
            else
            {
                local_final = inertialization_positive ? local_anim1 : local_anim0;
            }
        }
        else if (perform_dead_blending)
        {
            if (dead_blend_trigger)
            {
                local_dead_blending_transition = local_final;
                dead_blend_active = true;
                dead_blend_time = 0.0f;
                dead_blend_trigger = false;
            }
            
            if (dead_blend_active)
            {
                dead_blend_time = clampf(dead_blend_time + dt, 0.0f, dead_blend_duration);
                
                op_dead_blending(
                    local_final,
                    local_dead_blending_extrapolation,
                    local_dead_blending_transition,
                    dead_blend_positive ? local_anim1 : local_anim0,
                    dead_blending_extrapolation_halflife,
                    dead_blend_time,
                    dead_blend_duration,
                    dt);
                    
                if (dead_blend_time >= dead_blend_duration)
                {
                    dead_blend_active = false;
                }
            }
            else
            {
                local_final = dead_blend_positive ? local_anim1 : local_anim0;
            }
        }
        else if (perform_blending)
        {
            // Blending
            
            blend_time = clampf(blend_time + (blend_positive ? dt : -dt), 0.0f, blend_duration);
            
            blend_alpha = smoothstep_blend_alpha(
                blend_time,
                blend_duration);

            blend_alpha_vel = smoothstep_blend_alpha_dx(
                blend_time,
                blend_duration,
                blend_positive ? dt : -dt);
            
            op_blend(
                local_final,
                local_anim0,
                local_anim1,
                blend_alpha,
                blend_alpha_vel,
                dt);
        }
        else
        {
            local_final = local_anim0;
        }
        
        // Additive
        
        if (perform_additive)
        {
            if (sample_cubic)
            {
                op_sample_cubic(
                    local_additive, 
                    additive_pos, 
                    additive_rot, 
                    additive_scl,
                    time_additive, 
                    dt);           
            }
            else
            {
                op_sample_linear(
                    local_additive, 
                    additive_pos, 
                    additive_rot, 
                    additive_scl,
                    time_additive, 
                    dt);
            }
            
            op_timescale(local_additive, playback_dt / dt);
            
            additive_time = clampf(additive_time + (additive_positive ? dt : -dt), 0.0f, additive_duration);
            
            additive_alpha = smoothstep_blend_alpha(
                additive_time,
                additive_duration);
            
            additive_alpha_vel = smoothstep_blend_alpha_dx(
                additive_time,
                additive_duration,
                additive_positive ? dt : -dt);
            
            if (additive_local)
            {
                op_additive_local_inplace(
                    local_final,
                    local_additive,
                    additive_alpha,
                    additive_alpha_vel,
                    dt);
            }
            else
            {
                op_additive_inplace(
                    local_final,
                    local_additive,
                    additive_alpha,
                    additive_alpha_vel,
                    dt);   
            }
        }
        
        // Accumulate root
        
        local_root = local_final;
        
        root_vel = quat_mul_vec3(root_rot, local_root.vel(0));
        root_ang = quat_mul_vec3(root_rot, local_root.ang(0));
        root_pos = root_pos + dt * root_vel;
        root_rot = quat_mul(quat_from_scaled_angle_axis(dt * root_ang), root_rot);
        
        local_root.pos(0) = root_pos;
        local_root.rot(0) = root_rot;
        local_root.vel(0) = root_vel;
        local_root.ang(0) = root_ang;
        
        // Lookat
        
        if (lookat_enabled)
        {
            if (lookat_velocities)
            {
                pose temp_global0 = global_curr;
                op_fk(temp_global0, local_root, db.bone_parents);
                
                op_lookat(
                    local_root,
                    temp_global0,
                    lookat_target_curr,
                    lookat_target_vel);
            }
            else
            {
                pose temp_global0 = global_curr;
                op_fk(temp_global0, local_root, db.bone_parents);

                op_lookat_static(
                    local_root,
                    temp_global0,
                    lookat_target_curr);
            }
        }
        
        // IK

        if (ik_enabled)
        {
            if (ik_velocities)
            {
                pose temp_local0 = local_root;
                pose temp_global0 = global_curr;
                pose temp_global1 = global_curr;
                
                op_two_bone_ik(
                    local_root,
                    temp_local0,
                    temp_global0,
                    temp_global1,
                    db.bone_parents,
                    ik_target_curr,
                    ik_target_vel,
                    dt);
            }
            else
            {
                pose temp_global0 = global_curr;
                op_fk(temp_global0, local_root, db.bone_parents);

                op_two_bone_ik_static(
                    local_root,
                    temp_global0,
                    ik_target_curr);
            }
        }
        
        // Done
        
        local_prev = local_curr;
        local_curr = local_root;

        // Do FK
        
        global_prev = global_curr;
        
        op_fk(
            global_curr,
            local_curr,
            db.bone_parents);
        
        // Finite Diff
        
        local_diff = local_curr;        
        op_finite_diff(
            local_diff,
            local_curr,
            local_prev,
            dt);
          
        global_diff = global_curr;
        op_finite_diff(
            global_diff,
            global_curr,
            global_prev,
            dt);
            
        // Do BK
        
        op_bk(
            local_back,
            global_diff,
            db.bone_parents);

        // Render
        
        BeginDrawing();
        ClearBackground(RAYWHITE);
        
        BeginMode3D(camera);
        
        deform_character_mesh(
            character_mesh, 
            character_data, 
            global_curr.pos, 
            global_curr.rot,
            global_curr.scl,
            db.bone_parents);
        
        DrawGrid(20, 1.0f);
        draw_axis(vec3(), quat());
        
        DrawModel(ground_plane_model, (Vector3){0.0f, -0.01f, 0.0f}, 1.0f, WHITE);        
        DrawModel(character_model, (Vector3){0.0f, 0.0f, 0.0f}, 1.0f, RAYWHITE);
        
        if (ik_enabled)
        {
            DrawSphereWires(
                { ik_target_curr.x, ik_target_curr.y, ik_target_curr.z },
                0.025, 4, 8,
                VIOLET);
                
            DrawLine3D(
                { ik_target_curr.x, ik_target_curr.y, ik_target_curr.z },
                { ik_target_curr.x + 0.1f * ik_target_vel.x, ik_target_curr.y + 0.1f * ik_target_vel.y, ik_target_curr.z + 0.1f * ik_target_vel.z },
                VIOLET);
        }
        
        if (lookat_enabled)
        {
            DrawSphereWires(
                { lookat_target_curr.x, lookat_target_curr.y, lookat_target_curr.z },
                0.025, 4, 8,
                VIOLET);
                
            DrawLine3D(
                { global_curr.pos(Bone_Head).x, global_curr.pos(Bone_Head).y, global_curr.pos(Bone_Head).z },
                { lookat_target_curr.x, lookat_target_curr.y, lookat_target_curr.z },
                VIOLET);
                
            DrawLine3D(
                { lookat_target_curr.x, lookat_target_curr.y, lookat_target_curr.z },
                { lookat_target_curr.x + 0.1f * lookat_target_vel.x, lookat_target_curr.y + 0.1f * lookat_target_vel.y, lookat_target_curr.z + 0.1f * lookat_target_vel.z },
                VIOLET);
        }
        
        //////////////
        
        rlDrawRenderBatchActive();        
        rlDisableDepthTest();

        for (int i = 0; i < global_curr.pos.size; i++)
        {
            float diff_offset = 0.01f;
            float velocity_scale = 0.1f;
            vec3 p0 = global_curr.pos(i);
            vec3 p1 = global_curr.pos(i) + velocity_scale * global_curr.vel(i);
            vec3 p2 = global_curr.pos(i) + velocity_scale * global_diff.vel(i);
            vec3 p3 = global_curr.pos(i) + velocity_scale * global_curr.ang(i);
            vec3 p4 = global_curr.pos(i) + velocity_scale * global_diff.ang(i);
            vec3 p5 = global_curr.pos(i) + velocity_scale * quat_mul_vec3(global_curr.rot(i), local_curr.ang(i));
            vec3 p6 = global_curr.pos(i) + velocity_scale * quat_mul_vec3(global_curr.rot(i), local_diff.ang(i));
            vec3 p7 = global_curr.pos(i) + velocity_scale * quat_mul_vec3(global_curr.rot(i), local_back.ang(i));
            vec3 p8 = global_curr.pos(i) + global_curr.svl(i);
            vec3 p9 = global_curr.pos(i) + global_diff.svl(i);
            vec3 p10 = global_curr.pos(i) + local_back.svl(i);
            vec3 p11 = global_curr.pos(i) + local_back.svl(i);
            
            DrawSphereWires({ p0.x, p0.y, p0.z }, 0.01f, 6, 8, GRAY);
            if (db.bone_parents(i) != -1)
            {
                vec3 par = global_curr.pos(db.bone_parents(i));
                DrawLine3D({ p0.x, p0.y, p0.z }, { par.x, par.y, par.z }, GRAY);
            }
            
            // World Space Linear Velocities
            DrawLine3D({ p0.x, p0.y, p0.z }, { p1.x, p1.y, p1.z }, RED);
            DrawLine3D({ p0.x, p0.y + diff_offset, p0.z }, { p2.x, p2.y + diff_offset, p2.z }, BLUE); 
            
            // World Space Angular Velocities
            //DrawLine3D({ p0.x, p0.y, p0.z }, { p3.x, p3.y, p3.z }, RED);
            //DrawLine3D({ p0.x, p0.y + diff_offset, p0.z }, { p4.x, p4.y + diff_offset, p4.z }, BLUE); 
            
            // World Space Scalar Velocities
            //DrawLine3D({ p0.x, p0.y, p0.z }, { p8.x, p8.y, p8.z }, RED);
            //DrawLine3D({ p0.x, p0.y + diff_offset, p0.z }, { p9.x, p9.y + diff_offset, p9.z }, BLUE); 
            
            // Local Angular Velocities
            //DrawLine3D({ p0.x, p0.y, p0.z }, { p5.x, p5.y, p5.z }, RED);
            //DrawLine3D({ p0.x, p0.y + diff_offset, p0.z }, { p6.x, p6.y + diff_offset, p6.z }, BLUE);
            
            // Backward Kinematics Local Angular Velocities
            //DrawLine3D({ p0.x, p0.y, p0.z }, { p7.x, p7.y, p7.z }, RED);
            //DrawLine3D({ p0.x, p0.y + diff_offset, p0.z }, { p6.x, p6.y + diff_offset, p6.z }, BLUE); 
            
            // Backward Kinematics Scalar Velocities
            //DrawLine3D({ p0.x, p0.y, p0.z }, { p10.x, p10.y, p10.z }, RED);
            //DrawLine3D({ p0.x, p0.y + diff_offset, p0.z }, { p11.x, p11.y + diff_offset, p11.z }, BLUE); 
        }
        
        rlDrawRenderBatchActive();
        rlEnableDepthTest();
        
        //////////////

        EndMode3D();

        // UI
        
        float ui_blend_hei = 20;
        
        /*
        if (GuiButton((Rectangle){ 20, ui_blend_hei, 40, 20 }, "blend"))
        {
            blend_positive = !blend_positive;
        }
        
        GuiLabel((Rectangle){ 80, ui_blend_hei, 200, 20 }, TextFormat("%4.2f", blend_alpha));
        GuiSliderBar((Rectangle){ 100, ui_blend_hei + 30, 120, 20 }, "blend duration", TextFormat("%5.3f", blend_duration), &blend_duration, 0.0f, 2.0f);
        */
        
        /*
        if (GuiButton((Rectangle){ 20, ui_blend_hei, 60, 20 }, "additive"))
        {
            additive_positive = !additive_positive;
        }
        
        GuiLabel((Rectangle){ 100, ui_blend_hei, 200, 20 }, TextFormat("%4.2f", additive_alpha));
        GuiSliderBar((Rectangle){ 100, ui_blend_hei + 30, 120, 20 }, "blend duration", TextFormat("%5.3f", additive_duration), &additive_duration, 0.0f, 2.0f);
        */
        
        /*
        if (GuiButton((Rectangle){ 20, ui_blend_hei, 80, 20 }, "inertialization"))
        {
            inertialization_positive = !inertialization_positive;
            inertialization_trigger = true;
        }
        
        GuiLabel((Rectangle){ 120, ui_blend_hei, 200, 20 }, TextFormat("%4.2f", inertialization_active ? inertialization_time : 0.0f));
        GuiSliderBar((Rectangle){ 140, ui_blend_hei + 30, 120, 20 }, "inertialization duration", TextFormat("%5.3f", inertialization_duration), &inertialization_duration, 0.0f, 2.0f);
        */
        
        /*
        if (GuiButton((Rectangle){ 20, ui_blend_hei, 80, 20 }, "dead blend"))
        {
            dead_blend_positive = !dead_blend_positive;
            dead_blend_trigger = true;
        }
        
        float dead_blend_alpha = smoothstep_blend_alpha(dead_blend_time, dead_blend_duration);
        
        GuiLabel((Rectangle){ 120, ui_blend_hei, 200, 20 }, TextFormat("%4.2f", dead_blend_active ? dead_blend_alpha : 0.0f));
        GuiSliderBar((Rectangle){ 100, ui_blend_hei + 30, 120, 20 }, "blend duration", TextFormat("%5.3f", dead_blend_duration), &dead_blend_duration, 0.0f, 2.0f);
        */
        
        //GuiCheckBox((Rectangle){ 20, ui_blend_hei, 20, 20 }, "follow path", &lookat_target_path);
        
        //GuiCheckBox((Rectangle){ 20, ui_blend_hei, 20, 20 }, "follow path", &ik_target_path);

        EndDrawing();
        
    };
    
#if defined(PLATFORM_WEB)
    std::function<void()> u{update_func};
    emscripten_set_main_loop_arg(update_callback, &u, 0, 1);
#else
    while (!WindowShouldClose())
    {
        update_func();
    }
#endif

    // Unload stuff and finish
    UnloadModel(character_model);
    UnloadModel(ground_plane_model);
    UnloadShader(character_shader);
    UnloadShader(ground_plane_shader);

    CloseWindow();

    return 0;
}