#include "animation.h"
#include "tesselation.h"

// compute the frame from an animation
frame3f animate_compute_frame(FrameAnimation* animation, int time) {
    // grab keyframe interval
    auto interval = 0;
    for(auto t : animation->keytimes) if(time < t) break; else interval++;
    interval--;
    // get translation and rotation matrices
    auto t = float(time-animation->keytimes[interval])/float(animation->keytimes[interval+1]-animation->keytimes[interval]);
    auto m_t = translation_matrix(animation->translation[interval]*(1-t)+animation->translation[interval+1]*t);
    auto m_rz = rotation_matrix(animation->rotation[interval].z*(1-t)+animation->rotation[interval+1].z*t,z3f);
    auto m_ry = rotation_matrix(animation->rotation[interval].y*(1-t)+animation->rotation[interval+1].y*t,y3f);
    auto m_rx = rotation_matrix(animation->rotation[interval].x*(1-t)+animation->rotation[interval+1].x*t,x3f);
    // compute combined xform matrix
    auto m = m_t * m_rz * m_ry * m_rx;
    // return the transformed frame
    return transform_frame(m, animation->rest_frame);
}

// update mesh frames for animation
void animate_frame(Scene* scene) {
    // YOUR CODE GOES HERE ---------------------
    // foreach mesh
    for (Mesh* mesh : scene->meshes) {
        // if not animation, continue
        if (!mesh->animation) {
            continue;
        }
        // update frame
        mesh->frame = animate_compute_frame(mesh->animation, scene->animation->time);
    }
    // foreach surface
    for (Surface* surf : scene->surfaces) {
        // if not animation, continue
        if (!surf->animation) {
            continue;
        }
        // update frame
        surf->frame = animate_compute_frame(surf->animation, scene->animation->time);
        // update the _display_mesh
        surf->_display_mesh->frame = surf->frame;
        
    }
}

// skinning scene
void animate_skin(Scene* scene) {
    // YOUR CODE GOES HERE ---------------------
    // foreach mesh
    for (Mesh* mesh : scene->meshes) {
        // if no skinning, continue
        if (! mesh->skinning) continue;
        // foreach vertex index
        for (int v_idx : range(mesh->pos.size())){
            // set pos/norm to zero
            mesh->pos[v_idx] = zero3f;
            mesh->norm[v_idx] = zero3f;
            // for each bone slot (0..3)
            for (int bone_n : range(0, 3)){
                // get bone weight and index
                float bone_weight = mesh->skinning->bone_weights[v_idx][bone_n];
                int bone_idx = mesh->skinning->bone_ids[v_idx][bone_n];
                
                // if index < 0, continue
                if (bone_idx < 0) continue;
                // grab bone xform
                mat4f bone_xform = mesh->skinning->bone_xforms[scene->animation->time][bone_idx];
                
                // update position and normal
                vec3f rest_pos = mesh->skinning->rest_pos[v_idx];
                vec3f rest_norm = mesh->skinning->rest_norm[v_idx];
                
                mesh->pos[v_idx] += bone_weight*transform_point(bone_xform, rest_pos);
                mesh->pos[v_idx] += bone_weight*transform_normal(bone_xform, rest_norm);
                
                
            }
            // normalize normal
            mesh->norm[v_idx] = normalize(mesh->norm[v_idx]);
        }
    }
}

// particle simulation
void simulate(Scene* scene) {
    // YOUR CODE GOES HERE ---------------------
    // for each mesh
    for(Mesh* mesh: scene->meshes) {
        // skip if no simulation
        if (! mesh->simulation) continue;
        
        // compute time per step
        float t = scene->animation->dt/scene->animation->simsteps;
        // foreach simulation steps
        for(int i = 0; i < scene->animation->simsteps; i++){
            // compute extenal forces (gravity)
            const float g = 9.81;
            float F = mesh->simulation->mass * g;
            // for each spring, compute spring force on points
                // compute spring distance and length
                // compute static force
                // accumulate static force on points
                // compute dynamic force
                // accumulate dynamic force on points
            // newton laws
                // if pinned, skip
                // acceleration
                // update velocity and positions using Euler's method
                // for each mesh, check for collision
                    // compute inside tests
                    // if quad
                        // compute local poisition
                        // perform inside test
                            // if inside, set position and normal
                        // else sphere
                        // inside test
                            // if inside, set position and normal
                    // if inside
                        // set particle position
                        // update velocity
        }
        // smooth normals if it has triangles or quads
    }
}

// scene reset
void animate_reset(Scene* scene) {
    scene->animation->time = 0;
    for(auto mesh : scene->meshes) {
        if(mesh->animation) {
            mesh->frame = mesh->animation->rest_frame;
        }
        if(mesh->skinning) {
            mesh->pos = mesh->skinning->rest_pos;
            mesh->norm = mesh->skinning->rest_norm;
        }
        if(mesh->simulation) {
            mesh->pos = mesh->simulation->init_pos;
            mesh->simulation->vel = mesh->simulation->init_vel;
            mesh->simulation->force.resize(mesh->simulation->init_pos.size());
        }
    }
}

// scene update
void animate_update(Scene* scene) {
    if(scene->animation->time >= scene->animation->length-1) {
        if(scene->animation->loop) animate_reset(scene);
        else return;
    } else scene->animation->time ++;
    animate_frame(scene);
    if(not scene->animation->gpu_skinning) animate_skin(scene);
    simulate(scene);
}
