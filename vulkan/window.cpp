#include <window.hpp>
#include <iostream>

Window::Window(int w, int h, std::string n) : width(w), height(h), name(n)
{
    glfwInit();
   // glfwWindowHint(GLFW_DECORATED, GLFW_FALSE);
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);

    window = glfwCreateWindow(width, height, name.c_str(), nullptr, nullptr);

}

void Window::createWindowSurface(VkInstance instance, VkSurfaceKHR *surface)
{
    if (glfwCreateWindowSurface(instance, window, nullptr, surface) != VK_SUCCESS)
    {
        throw std::runtime_error("failed to craete window surface");
    }
}





Window::~Window()
{
    glfwDestroyWindow(window);
    glfwTerminate();
}