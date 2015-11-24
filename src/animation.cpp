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
            for (int bone_n : range(0, 4)){
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
                mesh->norm[v_idx] += bone_weight*transform_normal(bone_xform, rest_norm);
                
                
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
    bool inside = false;
    vec3f g = scene->animation->gravity;
    vec3f spring_direction, static_force, spring_relative_vel, dynamic_force, new_pos, new_norm;
;
    float spring_length;
    int v_id1, v_id2;

    for(Mesh* mesh: scene->meshes) {
        // skip if no simulation
        if (! mesh->simulation) continue;
        
        // compute time per step
        float t = scene->animation->dt/scene->animation->simsteps;
        // foreach simulation steps
        
        for(int simstep = 0; simstep < scene->animation->simsteps; simstep++){
            // compute extenal forces (gravity)
            
            fill(mesh->simulation->force.begin(), mesh->simulation->force.end(), g);
            for(int force = 0; force<mesh->simulation->force.size(); force++){

                    mesh->simulation->force[force] *= mesh->simulation->mass[force*3];
            }
            
            // for each spring, compute spring force on points

            for (auto spring: mesh->simulation->springs) {
                
                // compute spring distance and length
                v_id1 = spring.ids.x;
                v_id2 = spring.ids.y;
                
                spring_length = length(mesh->pos[v_id2]-mesh->pos[v_id1]);
                spring_direction = normalize(mesh->pos[v_id2]-mesh->pos[v_id1]);
                // compute static force
                static_force = spring.ks*(spring_length-spring.restlength)*spring_direction;
                
                // accumulate static force on points
                mesh->simulation->force[v_id1] += static_force;
                mesh->simulation->force[v_id2] += -static_force;
                
                // compute dynamic force
                spring_relative_vel = mesh->simulation->vel[v_id2]-mesh->simulation->vel[v_id1];
                dynamic_force = spring.kd*(dot(spring_relative_vel, spring_direction))*spring_direction;
                // accumulate dynamic force on points
                mesh->simulation->force[v_id1] += dynamic_force;
                mesh->simulation->force[v_id2] += -dynamic_force;
                
            }
            // newton laws
            for(int point = 0; point < mesh->pos.size(); point++){
                // if pinned, skip
                if (mesh->simulation->pinned[point]) continue;
                
                
                // acceleration
                vec3f a = mesh->simulation->force[point]/mesh->simulation->mass[point*3];
                // update velocity and positions using Euler's method
                mesh->simulation->vel[point] += a*t;
                mesh->pos[point] += mesh->simulation->vel[point]*t+a*t*t/2;

                // for each mesh, check for collision
                for (Surface* collision_mesh: scene->surfaces) {
                    // compute inside tests
                    // if quad
                    
                    if (collision_mesh->isquad) {
                        // compute local poisition
                        vec3f local_pos = transform_point_inverse(collision_mesh->frame, mesh->pos[point]);
                        float r = collision_mesh->radius;
                        // perform inside test
                        // if inside, set position and normal
                        if (local_pos.x > -r && local_pos.x < r &&
                            local_pos.y > -r && local_pos.y < r &&
                            local_pos.z < 0 ) {
                            
                            new_pos = transform_point(collision_mesh->frame, vec3f(local_pos.x, local_pos.y, 0));
                            new_norm =  collision_mesh->frame.z;
                            inside = true;
                        }
                        else inside = false;
                        
                    } else {
                        // else sphere
                        vec3f c = collision_mesh->frame.o;
                        float r = collision_mesh->radius;
                        // inside test
                        // if inside, set position and normal
                        
                        if (length(mesh->pos[point]-c)<r) {
                            new_pos = r*normalize(mesh->pos[point]-c)+c;
                            new_norm = normalize(mesh->pos[point]-c);
                            inside = true;
                        }
                        else inside = false;
                        
                    }
                    
                    // if inside
                    if (inside) {
                        // set particle position
                        mesh->pos[point] = new_pos;
                        mesh->norm[point] = new_norm;
                        // update velocity
                        float d_p = scene->animation->bounce_dump.x;
                        float d_o = scene->animation->bounce_dump.y;
                        vec3f v = mesh->simulation->vel[point];
                        mesh->simulation->vel[point] = (v-dot(mesh->norm[point], v)*mesh->norm[point])*(1-d_p)+(-dot(mesh->norm[point], v)*mesh->norm[point])*(1-d_o);
                    }
                    
                }

            }
        }
        // smooth normals if it has triangles or quads
        if(mesh->triangle.size()!=0 || mesh->quad.size()!=0)
            smooth_normals(mesh);
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
