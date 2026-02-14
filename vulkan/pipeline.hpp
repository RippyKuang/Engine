#pragma once

#include "device.hpp"
#include "swap_chain.hpp"
#include <renderer.hpp>
#include "model.hpp"
#include <string>
#include <vector>
#include <fstream>

struct PipelineConfigInfo
{
};
class PipeLine
{
public:
    PipeLine(Device &device,
             VkRenderPass renderer, 
            VkPipelineLayout pipelineLayout,
             const std::string &vertFilepath,
             const std::string &fragFilepath);
    ~PipeLine();
    PipeLine(const PipeLine &) = delete;
    PipeLine &operator=(const PipeLine &) = delete;
    void bind(VkCommandBuffer commandBuffer); 
private:
    Device &device;
    VkRenderPass renderPass;
    VkPipeline graphicsPipeline;
    VkPipelineLayout pipelineLayout;
    VkShaderModule createShaderModule(const std::vector<char> &code);
    void createGraphicsPipeLine(const std::string &vertFilepath,
                                const std::string &fragFilepath);
    static std::vector<char> readFile(const std::string &filename);
};