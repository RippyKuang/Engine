#include "device.hpp"
#include "swap_chain.hpp"
#include "pipeline.hpp"
#include "model.hpp"
#include "items.h"
#include <memory>
#include <array>
#include <thread>

class EngineApplication
{
public:
    static constexpr int WIDTH = 1280;
    static constexpr int HEIGHT = 1024;

    EngineApplication();
    ~EngineApplication();

    EngineApplication(const EngineApplication &) = delete;
    EngineApplication &operator=(const EngineApplication &) = delete;
    void run();
    void update(std::vector<Engine::Matrix<float, 4, 4>> &pose);
    void loadWorld(const std::vector<Engine::Link *>);

private:
    std::thread daemon;
    bool daemon_running = false;
    std::mutex m;
    Window window{WIDTH, HEIGHT, "VkEngine"};
    Device device{window};
    Renderer renderer{window, device};
    VkExtent2D swapChainExtent;

    VkDescriptorSetLayout descriptorSetLayout;
    VkDescriptorPool descriptorPool;
    std::vector<VkDescriptorSet> descriptorSets;
    VkPipelineLayout pipelineLayout;

    std::unique_ptr<PipeLine> pipeline;
    std::unique_ptr<Model> model;

    std::vector<VkBuffer> uniformBuffers;
    std::vector<VkDeviceMemory> uniformBuffersMemory;
    std::vector<glm::mat4> pose;
    std::vector<void *> uniformBuffersMapped;
    std::vector<uint32_t> offset; 
    uint32_t objCount = 0; 
    void createDescriptorSetLayout();
    void createPipelineLayout();
    void createUniformBuffers();
    void createDescriptorPool();
    void daemon_run();
    void parseLink(Engine::Link *link, std::vector<Model::Vertex>&, std::vector<INDICES_TYPE>& indices,uint32_t offset = 0);
};