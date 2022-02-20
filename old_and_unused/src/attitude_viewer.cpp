#include <glad/glad.h> // ensure to include GLAD before GLFW
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>

#include <iostream>
#include <sstream>
#include <fstream>

#include <chrono>
#include <thread>
#include <pthread.h>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "attitude_viewer.hpp"
#include "rigidbody_sim.hpp"

extern "C"
{
#include "rigidbody.h"
}

void renderPipeline(PhysicsSimProperties* sim_properties, double timestep)
{
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
#ifdef __APPLE__
    // needed for Mac OS X
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    GLFWwindow *window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Attitude Viewer", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return;
    }
    glfwMakeContextCurrent(window);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        glfwTerminate();
        return;
    }

    // flips to get correct orientation of loaded images -> (0, 0) is bottom-left
    stbi_set_flip_vertically_on_load(1);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE); // only CCW faces visible

    glViewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);

    // prepares satellite VAO
    unsigned int sat_vao, sat_vbo, sat_ebo;
    glGenVertexArrays(1, &sat_vao);
    glGenBuffers(1, &sat_vbo);
    glGenBuffers(1, &sat_ebo);

    glBindVertexArray(sat_vao);

    glBindBuffer(GL_ARRAY_BUFFER, sat_vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(sat_vertices), sat_vertices, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, sat_ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(sat_indices), sat_indices, GL_STATIC_DRAW);

    // position attribute (first three values of point data)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);
    // texture coord attribute (last two values of point data)
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    // texture filtering parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    int width, height, n_channels;
    unsigned char *data = stbi_load("assets/cubesat_model_tex.png", &width, &height, &n_channels, 0);
    if (data)
    {
        // imported images have alpha channel -> note usage of GL_RGBA when passing data
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);
    }
    else
        std::cout << "Failed to load texture" << std::endl;
    stbi_image_free(data);

    // sets up the satellite shader
    const unsigned int sat_shader_prog = generateProgram("assets/sat.vs", "assets/sat.fs");
    glUseProgram(sat_shader_prog);
    glUniform1i(glGetUniformLocation(sat_shader_prog, "tex"), 0); // sets to use texture in shader program

    // prepares origin VAO
    unsigned int origin_vao, origin_vbo;
    glGenVertexArrays(1, &origin_vao);
    glGenBuffers(1, &origin_vbo);

    glBindVertexArray(origin_vao);

    glBindBuffer(GL_ARRAY_BUFFER, origin_vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(origin_vertices), origin_vertices, GL_STATIC_DRAW);

    // position attribute (three values of point data)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);

    const unsigned int origin_shader_prog = generateProgram("assets/origin.vs", "assets/origin.fs");
    glUseProgram(origin_shader_prog);

    glLineWidth(3.0f); // ensures the origin is rendered with thicker lines

    // only calculate view and projection once
    glm::mat4 view = glm::lookAt(
        glm::vec3(3.0f, 3.0f, 3.0f), // camera position
        glm::vec3(0.0f, 0.0f, 0.0f), // camera target position
        glm::vec3(0.0f, 1.0f, 0.0f)  // up vector in world space
    );
    glm::mat4 projection = glm::perspective(glm::radians(55.0f), (float)WINDOW_WIDTH / (float)WINDOW_HEIGHT, 0.1f, 100.0f);
    // glm::mat4 projection = glm::ortho(-4.0f, 4.0f, -4.0f, 4.0f, 0.1f, 100.0f);

    glm::mat4 model_origin_x = glm::mat4(1.0f); // vertices already set along x
    glm::mat4 model_origin_y = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f));
    glm::mat4 model_origin_z = glm::rotate(glm::mat4(1.0f), glm::radians(-90.0f), glm::vec3(0.0f, 1.0f, 0.0f));

    while (!glfwWindowShouldClose(window))
    {
        auto start = std::chrono::high_resolution_clock::now();

        // handle inputs

        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // render the origin ahead of satellite
        glUseProgram(origin_shader_prog);

        glUniformMatrix4fv(glGetUniformLocation(origin_shader_prog, "view"), 1, GL_FALSE, &view[0][0]);
        glUniformMatrix4fv(glGetUniformLocation(origin_shader_prog, "projection"), 1, GL_FALSE, &projection[0][0]);

        glBindVertexArray(origin_vao);

        // draws x-axis at origin
        glUniformMatrix4fv(glGetUniformLocation(origin_shader_prog, "model"), 1, GL_FALSE, &model_origin_x[0][0]);
        glUniform4fv(glGetUniformLocation(origin_shader_prog, "color"), 1, &color_origin_x[0]);
        glDrawArrays(GL_LINES, 0, 2);

        // draws y-axis at origin
        glUniformMatrix4fv(glGetUniformLocation(origin_shader_prog, "model"), 1, GL_FALSE, &model_origin_y[0][0]);
        glUniform4fv(glGetUniformLocation(origin_shader_prog, "color"), 1, &color_origin_y[0]);
        glDrawArrays(GL_LINES, 0, 2);

        // draws z-axis at origin
        glUniformMatrix4fv(glGetUniformLocation(origin_shader_prog, "model"), 1, GL_FALSE, &model_origin_z[0][0]);
        glUniform4fv(glGetUniformLocation(origin_shader_prog, "color"), 1, &color_origin_z[0]);
        glDrawArrays(GL_LINES, 0, 2);

        // render satellite
        glUseProgram(sat_shader_prog);

        glUniformMatrix4fv(glGetUniformLocation(sat_shader_prog, "view"), 1, GL_FALSE, &view[0][0]);
        glUniformMatrix4fv(glGetUniformLocation(sat_shader_prog, "projection"), 1, GL_FALSE, &projection[0][0]);

        glBindVertexArray(sat_vao);

        pthread_mutex_lock(&sim_properties->rbody_lock);
        // calculate model matrix (local to world pos) for satellite rigidbody
        glm::mat4 model_sat = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, 0.0f)); // no translation applied
        model_sat = model_sat * glm::toMat4(glm::quat(sim_properties->rbody.orient.w, glm::vec3(sim_properties->rbody.orient.v[0], sim_properties->rbody.orient.v[1], sim_properties->rbody.orient.v[2])));
        // model_sat = glm::rotate(model_sat, glm::vec3(rbody->orient.v[0], rbody->orient.)); // applies rotation
        pthread_mutex_unlock(&sim_properties->rbody_lock);

        glUniformMatrix4fv(glGetUniformLocation(sat_shader_prog, "model"), 1, GL_FALSE, &model_sat[0][0]);

        glDrawElements(GL_TRIANGLES, 60, GL_UNSIGNED_INT, 0);

        glfwSwapBuffers(window);
        glfwPollEvents();

        auto stop = std::chrono::high_resolution_clock::now();

        // enforces the timestep
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
        auto wait_duration = std::chrono::duration<double>(timestep) - duration;

        std::this_thread::sleep_for(wait_duration);
    }

    glDeleteVertexArrays(1, &sat_vao);
    glDeleteBuffers(1, &sat_vao);
    glDeleteBuffers(1, &sat_ebo);

    glfwTerminate(); // free resources
    return;
}

void framebufferSizeCallback(GLFWwindow *window, int width, int height)
{
    glViewport(0, 0, width, height);
}

unsigned int generateProgram(const std::string vertex_path, const std::string fragment_path)
{
    unsigned int vertex_shader, fragment_shader;
    vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);

    // loads the source for the shaders
    std::ifstream vertex_source_file(vertex_path);
    std::ifstream fragment_source_file(fragment_path);

    std::stringstream vertex_source_buffer, fragment_source_buffer;
    vertex_source_buffer << vertex_source_file.rdbuf();
    fragment_source_buffer << fragment_source_file.rdbuf();

    // files are no longer needed
    vertex_source_file.close();
    fragment_source_file.close();

    // needs to have a pointer to the const char* the values for glShaderSource
    const std::string vertex_source(vertex_source_buffer.str());
    const std::string fragment_source(fragment_source_buffer.str());
    const char *vertex_source_ptr = vertex_source.c_str();
    const char *fragment_source_ptr = fragment_source.c_str();

    glShaderSource(vertex_shader, 1, &vertex_source_ptr, NULL);
    glCompileShader(vertex_shader);

    glShaderSource(fragment_shader, 1, &fragment_source_ptr, NULL);
    glCompileShader(fragment_shader);

    fragment_source_file.close();

    int success;
    char info_log[512];
    glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        glGetShaderInfoLog(vertex_shader, 512, NULL, info_log);
        std::cout << "Error compiling vertex shader: " << vertex_path << "\n"
                  << info_log << std::endl;
    }

    glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        glGetShaderInfoLog(fragment_shader, 512, NULL, info_log);
        std::cout << "Error compiling fragment shader: " << fragment_path << "\n"
                  << info_log << std::endl;
    }

    const unsigned int sat_shader_prog = glCreateProgram();

    glAttachShader(sat_shader_prog, vertex_shader);
    glAttachShader(sat_shader_prog, fragment_shader);
    glLinkProgram(sat_shader_prog);

    glGetProgramiv(sat_shader_prog, GL_LINK_STATUS, &success);
    if (!success)
    {
        glGetProgramInfoLog(sat_shader_prog, 512, NULL, info_log);
        std::cout << "Error linking shader program:\n"
                  << info_log << std::endl;
    }

    // no longer needed
    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);

    return sat_shader_prog;
}