#ifndef ATTITUDE_VIEWER_HPP
#define ATTITUDE_VIEWER_HPP

#include <glad/glad.h> // GLAD must be included before GLFW
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>

#include <string>
#include <pthread.h>

extern "C"
{
#include "rigidbody.h"
}

#include "rigidbody_sim.hpp"

// define global constants for window size
#define WINDOW_WIDTH 1200
#define WINDOW_HEIGHT 750

// satellite model
const float sat_pixel_width = 1.0f / 256;
const float sat_pixel_height = 1.0f / 128;

const float sat_vertices[] = {
    // front face
    -0.5, -0.5, 0.5, 0, 0,
    -0.5, 0.5, 0.5, 0, 64 * sat_pixel_height,
    0.5, 0.5, 0.5, 64 * sat_pixel_width, 64 * sat_pixel_height,
    0.5, -0.5, 0.5, 64 * sat_pixel_width, 0,

    // right face
    0.5, -0.5, 0.5, 64 * sat_pixel_width, 0,
    0.5, 0.5, 0.5, 64 * sat_pixel_width, 64 * sat_pixel_height,
    0.5, 0.5, -0.5, 128 * sat_pixel_width, 64 * sat_pixel_height,
    0.5, -0.5, -0.5, 128 * sat_pixel_width, 0,

    // back face
    0.5, -0.5, -0.5, 128 * sat_pixel_width, 0,
    0.5, 0.5, -0.5, 128 * sat_pixel_width, 64 * sat_pixel_height,
    -0.5, 0.5, -0.5, 192 * sat_pixel_width, 64 * sat_pixel_height,
    -0.5, -0.5, -0.5, 192 * sat_pixel_width, 0,

    // left face
    -0.5, -0.5, -0.5, 192 * sat_pixel_width, 0,
    -0.5, 0.5, -0.5, 192 * sat_pixel_width, 64 * sat_pixel_height,
    -0.5, 0.5, 0.5, 256 * sat_pixel_width, 64 * sat_pixel_height,
    -0.5, -0.5, 0.5, 256 * sat_pixel_width, 0,

    // top face
    -0.5, 0.5, 0.5, 0, 64 * sat_pixel_height,
    -0.5, 0.5, -0.5, 0, 128 * sat_pixel_height,
    0.5, 0.5, -0.5, 64 * sat_pixel_width, 128 * sat_pixel_height,
    0.5, 0.5, 0.5, 64 * sat_pixel_width, 64 * sat_pixel_height,

    // bottom face
    -0.5, -0.5, -0.5, 64 * sat_pixel_width, 64 * sat_pixel_height,
    -0.5, -0.5, 0.5, 64 * sat_pixel_width, 128 * sat_pixel_height,
    0.5, -0.5, 0.5, 128 * sat_pixel_width, 128 * sat_pixel_height,
    0.5, -0.5, -0.5, 128 * sat_pixel_width, 64 * sat_pixel_height,

    // solar panel left - top
    -1.5, -0.2, 0.5, 128 * sat_pixel_width, 64 * sat_pixel_height,
    -1.5, 0, -0.5, 128 * sat_pixel_width, 128 * sat_pixel_height,
    -0.5, 0, -0.5, 192 * sat_pixel_width, 128 * sat_pixel_height,
    -0.5, -0.2, 0.5, 192 * sat_pixel_width, 64 * sat_pixel_height,

    // solar panel left - bottom
    -1.5, 0, -0.5, 128 * sat_pixel_width, 64 * sat_pixel_height,
    -1.5, -0.2, 0.5, 128 * sat_pixel_width, 128 * sat_pixel_height,
    -0.5, -0.2, 0.5, 192 * sat_pixel_width, 128 * sat_pixel_height,
    -0.5, 0, -0.5, 192 * sat_pixel_width, 64 * sat_pixel_height,

    // solar panel right - top
    0.5, -0.2, 0.5, 128 * sat_pixel_width, 64 * sat_pixel_height,
    0.5, 0, -0.5, 128 * sat_pixel_width, 128 * sat_pixel_height,
    1.5, 0, -0.5, 192 * sat_pixel_width, 128 * sat_pixel_height,
    1.5, -0.2, 0.5, 192 * sat_pixel_width, 64 * sat_pixel_height,

    // solar panel right - bottom
    0.5, 0, -0.5, 128 * sat_pixel_width, 64 * sat_pixel_height,
    0.5, -0.2, 0.5, 128 * sat_pixel_width, 128 * sat_pixel_height,
    1.5, -0.2, 0.5, 192 * sat_pixel_width, 128 * sat_pixel_height,
    1.5, 0, -0.5, 192 * sat_pixel_width, 64 * sat_pixel_height};

const unsigned int sat_indices[] = {
    // front face
    2, 1, 0,
    3, 2, 0,

    // right face
    6, 5, 4,
    7, 6, 4,

    // back face
    10, 9, 8,
    11, 10, 8,

    // left face
    14, 13, 12,
    15, 14, 12,

    // top face
    18, 17, 16,
    19, 18, 16,

    // bottom face
    22, 21, 20,
    23, 22, 20,

    // solar panel left - top
    26, 25, 24,
    27, 26, 24,

    // solar panel left - bottom
    30, 29, 28,
    31, 30, 28,

    // solar panel right - top
    34, 33, 32,
    35, 34, 32,

    // solar panel right - bottom
    38, 37, 36,
    39, 38, 36};

// a colored line extending from 0.0 to 3.0 on each axis
const glm::vec4 color_origin_x(1.0f, 0.0f, 0.0f, 1.0f);
const glm::vec4 color_origin_y(0.0f, 1.0f, 0.0f, 1.0f);
const glm::vec4 color_origin_z(0.0f, 0.0f, 1.0f, 1.0f);

const float origin_vertices[] = {
    0.0f, 0.0f, 0.0f,
    2.0f, 0.0f, 0.0f};

void framebufferSizeCallback(GLFWwindow *window, int width, int height);
unsigned int generateProgram(const std::string vertex_path, const std::string fragment_path);

void renderPipeline(PhysicsSimProperties* sim_properties, double timestep);

#endif