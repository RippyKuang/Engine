#pragma once
#include <device.hpp>
#include <cstdint>
#include <limits>
#include <algorithm>

class SwapChain
{
public:
    static constexpr int MAX_FRAMES_IN_FLIGHT = 2;
    SwapChain(Device &deviceRef, VkExtent2D extent);
    ~SwapChain();

    SwapChain(const SwapChain &) = delete;
    SwapChain &operator=(const SwapChain &) = delete;
    VkRenderPass getRenderPass() { return renderPass; }
    VkFramebuffer getFramebuffer(int i) { return swapChainFramebuffers[i]; }
    VkExtent2D getSwapChainExtent() { return swapChainExtent; }
    size_t imageCount() { return swapChainImages.size(); }
    VkResult acquireNextImage(uint32_t *imageIndex);
    VkResult submitCommandBuffers(const VkCommandBuffer *buffers, uint32_t *imageIndex);
    VkFormat findDepthFormat();

private:
    uint32_t currentFrame = 0;
    Device &device;
    VkExtent2D windowExtent;
    VkSwapchainKHR swapChain;
    std::vector<VkImage> swapChainImages;
    VkFormat swapChainImageFormat;
    VkExtent2D swapChainExtent;
    VkRenderPass renderPass;
    std::vector<VkImageView> swapChainImageViews;
    std::vector<VkFramebuffer> swapChainFramebuffers;

    std::vector<VkSemaphore> imageAvailableSemaphore;
    std::vector<VkSemaphore> renderFinishedSemaphore;
    std::vector<VkFence> inFlightFence;
    std::vector<VkFence> imagesInFlight;

    std::vector<VkImage> depthImages;
    std::vector<VkDeviceMemory> depthImageMemorys;
    std::vector<VkImageView> depthImageViews;

    void init();
    void createSwapChain();
    void createDepthResources();
    void createImageViews();
    void createImage(uint32_t width,
                     uint32_t height,
                     VkFormat format,
                     VkImageTiling tiling,
                     VkImageUsageFlags usage,
                     VkMemoryPropertyFlags properties,
                     VkImage &image,
                     VkDeviceMemory &imageMemory);
    void createRenderPass();
    void createFramebuffers();
    void createSyncObjects();

    VkSurfaceFormatKHR chooseSwapSurfaceFormat(const std::vector<VkSurfaceFormatKHR> &availableFormats);
    VkPresentModeKHR chooseSwapPresentMode(const std::vector<VkPresentModeKHR> &availablePresentModes);
    VkExtent2D chooseSwapExtent(const VkSurfaceCapabilitiesKHR &capabilities);
};