// Vulkan compute backend for particles, plus Vulkan device enumeration.
// The Vulkan loader is opened at runtime (volk), so Vexora runs on machines without Vulkan.
//
// Memory: particles live in a DEVICE_LOCAL buffer. The CPU writes spawned/killed slots into a
// host-visible staging mirror, and only those ranges are copied each step; full copies happen
// only on readback(). This keeps PCIe traffic proportional to emission, not particle count.

#include <volk.h>

#include <algorithm>
#include <cstring>
#include <mutex>

#include "ParticleBackend.h"
#include "generated/particles_comp_spv.h"

namespace vexora::physics::detail {
namespace {

constexpr uint32_t kLocalSize = 256;  // Must match local_size_x in shaders/particles.comp.

std::once_flag gVolkOnce;
VkResult gVolkResult = VK_ERROR_INITIALIZATION_FAILED;

bool loadVulkan(std::string& reason) {
    std::call_once(gVolkOnce, [] { gVolkResult = volkInitialize(); });
    if (gVolkResult != VK_SUCCESS) {
        reason = "Vulkan loader not found (install a GPU driver with Vulkan support)";
        return false;
    }
    return true;
}

std::string vkError(const char* what, VkResult r) { return std::string(what) + " failed (VkResult " + std::to_string(int(r)) + ")"; }

// Instance used only by this module. Each user creates its own so lifetimes stay independent.
VkResult createInstance(VkInstance& instance) {
    VkApplicationInfo app{VK_STRUCTURE_TYPE_APPLICATION_INFO};
    app.pApplicationName = "Vexora";
    app.apiVersion = VK_API_VERSION_1_1;
    VkInstanceCreateInfo ci{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
    ci.pApplicationInfo = &app;
#ifdef __APPLE__
    // MoltenVK is a portability implementation. Untested: no macOS machine was available.
    static const char* ext[] = {VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME};
    ci.flags |= VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR;
    ci.enabledExtensionCount = 1;
    ci.ppEnabledExtensionNames = ext;
#endif
    VkResult r = vkCreateInstance(&ci, nullptr, &instance);
    if (r == VK_SUCCESS) volkLoadInstanceOnly(instance);
    return r;
}

// Physical devices that have a compute queue, in driver order.
std::vector<std::pair<VkPhysicalDevice, uint32_t>> computeDevices(VkInstance instance) {
    uint32_t n = 0;
    vkEnumeratePhysicalDevices(instance, &n, nullptr);
    std::vector<VkPhysicalDevice> devices(n);
    vkEnumeratePhysicalDevices(instance, &n, devices.data());
    std::vector<std::pair<VkPhysicalDevice, uint32_t>> out;
    for (VkPhysicalDevice d : devices) {
        uint32_t qn = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(d, &qn, nullptr);
        std::vector<VkQueueFamilyProperties> q(qn);
        vkGetPhysicalDeviceQueueFamilyProperties(d, &qn, q.data());
        for (uint32_t i = 0; i < qn; ++i)
            if (q[i].queueFlags & VK_QUEUE_COMPUTE_BIT) {
                out.push_back({d, i});
                break;
            }
    }
    return out;
}

ComputeDeviceInfo describe(VkPhysicalDevice d, uint32_t index) {
    VkPhysicalDeviceProperties props;
    vkGetPhysicalDeviceProperties(d, &props);
    VkPhysicalDeviceMemoryProperties mem;
    vkGetPhysicalDeviceMemoryProperties(d, &mem);
    ComputeDeviceInfo info;
    info.backend = ComputeBackend::GPU;
    info.index = index;
    info.name = props.deviceName;
    switch (props.deviceType) {
        case VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU: info.type = DeviceType::DiscreteGpu; break;
        case VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU: info.type = DeviceType::IntegratedGpu; break;
        case VK_PHYSICAL_DEVICE_TYPE_VIRTUAL_GPU: info.type = DeviceType::VirtualGpu; break;
        default: info.type = DeviceType::SoftwareGpu; break;  // CPU / OTHER
    }
    for (uint32_t i = 0; i < mem.memoryHeapCount; ++i)
        if (mem.memoryHeaps[i].flags & VK_MEMORY_HEAP_DEVICE_LOCAL_BIT) info.memoryBytes = std::max<uint64_t>(info.memoryBytes, mem.memoryHeaps[i].size);
    info.apiVersion = "Vulkan " + std::to_string(VK_API_VERSION_MAJOR(props.apiVersion)) + "." +
                      std::to_string(VK_API_VERSION_MINOR(props.apiVersion)) + "." +
                      std::to_string(VK_API_VERSION_PATCH(props.apiVersion));
    return info;
}

class VulkanParticleBackend final : public ParticleBackend {
public:
    ~VulkanParticleBackend() override {
        if (device_) {
            vk_.vkDeviceWaitIdle(device_);
            if (fence_) vk_.vkDestroyFence(device_, fence_, nullptr);
            if (pool_) vk_.vkDestroyCommandPool(device_, pool_, nullptr);
            if (pipeline_) vk_.vkDestroyPipeline(device_, pipeline_, nullptr);
            if (pipelineLayout_) vk_.vkDestroyPipelineLayout(device_, pipelineLayout_, nullptr);
            if (descriptorPool_) vk_.vkDestroyDescriptorPool(device_, descriptorPool_, nullptr);
            if (setLayout_) vk_.vkDestroyDescriptorSetLayout(device_, setLayout_, nullptr);
            for (Buf* b : {&particles_, &staging_, &params_}) {
                if (b->mapped) vk_.vkUnmapMemory(device_, b->memory);
                if (b->buffer) vk_.vkDestroyBuffer(device_, b->buffer, nullptr);
                if (b->memory) vk_.vkFreeMemory(device_, b->memory, nullptr);
            }
            vk_.vkDestroyDevice(device_, nullptr);
        }
        if (instance_) vkDestroyInstance(instance_, nullptr);
    }

    Status init(uint32_t gpuIndex, uint32_t slots) {
        slots_ = slots;
        VkResult r = createInstance(instance_);
        if (r != VK_SUCCESS) return {ErrorCode::DeviceUnavailable, vkError("vkCreateInstance", r)};
        auto devices = computeDevices(instance_);
        if (gpuIndex >= devices.size()) return {ErrorCode::DeviceUnavailable, "GPU index out of range"};
        physical_ = devices[gpuIndex].first;
        const uint32_t family = devices[gpuIndex].second;

        const float priority = 1.0f;
        VkDeviceQueueCreateInfo qci{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
        qci.queueFamilyIndex = family;
        qci.queueCount = 1;
        qci.pQueuePriorities = &priority;
        VkDeviceCreateInfo dci{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
        dci.queueCreateInfoCount = 1;
        dci.pQueueCreateInfos = &qci;
        if ((r = vkCreateDevice(physical_, &dci, nullptr, &device_)) != VK_SUCCESS) return {ErrorCode::DeviceError, vkError("vkCreateDevice", r)};
        // Device-level functions come from this device's table, never from volk's globals.
        volkLoadDeviceTable(&vk_, device_);
        vk_.vkGetDeviceQueue(device_, family, 0, &queue_);

        const VkDeviceSize bytes = VkDeviceSize(slots) * sizeof(Particle);
        Status s = makeBuffer(particles_, bytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                              VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, 0);
        if (!s.ok()) return s;
        // Prefer cached host memory for fast readback; fall back to plain coherent memory.
        s = makeBuffer(staging_, bytes, VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                       VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, VK_MEMORY_PROPERTY_HOST_CACHED_BIT);
        if (!s.ok()) return s;
        s = makeBuffer(params_, sizeof(KernelParams), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                       VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, 0);
        if (!s.ok()) return s;
        std::memset(staging_.mapped, 0, size_t(bytes));  // All slots free (lifetime 0).

        if (!(s = makePipeline()).ok()) return s;

        VkCommandPoolCreateInfo pci{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
        pci.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        pci.queueFamilyIndex = family;
        if ((r = vk_.vkCreateCommandPool(device_, &pci, nullptr, &pool_)) != VK_SUCCESS) return {ErrorCode::DeviceError, vkError("vkCreateCommandPool", r)};
        VkCommandBufferAllocateInfo cai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
        cai.commandPool = pool_;
        cai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        cai.commandBufferCount = 1;
        if ((r = vk_.vkAllocateCommandBuffers(device_, &cai, &cmd_)) != VK_SUCCESS) return {ErrorCode::DeviceError, vkError("vkAllocateCommandBuffers", r)};
        VkFenceCreateInfo fci{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
        if ((r = vk_.vkCreateFence(device_, &fci, nullptr, &fence_)) != VK_SUCCESS) return {ErrorCode::DeviceError, vkError("vkCreateFence", r)};

        // Initialise the device buffer from the zeroed staging buffer.
        return submit([&](VkCommandBuffer c) {
            VkBufferCopy all{0, 0, bytes};
            vk_.vkCmdCopyBuffer(c, staging_.buffer, particles_.buffer, 1, &all);
        });
    }

    Particle* hostSlots() override { return static_cast<Particle*>(staging_.mapped); }

    Status update(const KernelParams& params, const SlotRanges& dirty) override {
        std::memcpy(params_.mapped, &params, sizeof(params));
        return submit([&](VkCommandBuffer c) {
            if (!dirty.empty()) {
                std::vector<VkBufferCopy> regions;
                regions.reserve(dirty.size());
                for (auto [first, last] : dirty)
                    regions.push_back({first * sizeof(Particle), first * sizeof(Particle), (last - first) * sizeof(Particle)});
                vk_.vkCmdCopyBuffer(c, staging_.buffer, particles_.buffer, uint32_t(regions.size()), regions.data());
                barrier(c, VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT,
                        VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
            }
            vk_.vkCmdBindPipeline(c, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_);
            vk_.vkCmdBindDescriptorSets(c, VK_PIPELINE_BIND_POINT_COMPUTE, pipelineLayout_, 0, 1, &set_, 0, nullptr);
            vk_.vkCmdDispatch(c, (slots_ + kLocalSize - 1) / kLocalSize, 1, 1);
        });
    }

    Status readback(std::vector<Particle>& out) override {
        const VkDeviceSize bytes = VkDeviceSize(slots_) * sizeof(Particle);
        Status s = submit([&](VkCommandBuffer c) {
            barrier(c, VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_TRANSFER_READ_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                    VK_PIPELINE_STAGE_TRANSFER_BIT);
            VkBufferCopy all{0, 0, bytes};
            vk_.vkCmdCopyBuffer(c, particles_.buffer, staging_.buffer, 1, &all);
        });
        if (!s.ok()) return s;
        if (!stagingCoherent_) {
            VkMappedMemoryRange range{VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE, nullptr, staging_.memory, 0, VK_WHOLE_SIZE};
            vk_.vkInvalidateMappedMemoryRanges(device_, 1, &range);
        }
        out.assign(hostSlots(), hostSlots() + slots_);
        return {};
    }

private:
    struct Buf {
        VkBuffer buffer = VK_NULL_HANDLE;
        VkDeviceMemory memory = VK_NULL_HANDLE;
        void* mapped = nullptr;
    };

    Status makeBuffer(Buf& b, VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags required, VkMemoryPropertyFlags preferred) {
        VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
        bci.size = size;
        bci.usage = usage;
        bci.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        VkResult r = vk_.vkCreateBuffer(device_, &bci, nullptr, &b.buffer);
        if (r != VK_SUCCESS) return {ErrorCode::DeviceError, vkError("vkCreateBuffer", r)};
        VkMemoryRequirements req;
        vk_.vkGetBufferMemoryRequirements(device_, b.buffer, &req);
        VkPhysicalDeviceMemoryProperties mem;
        vkGetPhysicalDeviceMemoryProperties(physical_, &mem);
        int chosen = -1;
        for (VkMemoryPropertyFlags want : {required | preferred, required})
            for (uint32_t i = 0; i < mem.memoryTypeCount && chosen < 0; ++i)
                if ((req.memoryTypeBits & (1u << i)) && (mem.memoryTypes[i].propertyFlags & want) == want) chosen = int(i);
        if (chosen < 0) return {ErrorCode::DeviceError, "No suitable GPU memory type"};
        if (&b == &staging_) stagingCoherent_ = mem.memoryTypes[chosen].propertyFlags & VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
        VkMemoryAllocateInfo mai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        mai.allocationSize = req.size;
        mai.memoryTypeIndex = uint32_t(chosen);
        if ((r = vk_.vkAllocateMemory(device_, &mai, nullptr, &b.memory)) != VK_SUCCESS)
            return {ErrorCode::DeviceError, vkError("vkAllocateMemory (GPU out of memory?)", r)};
        vk_.vkBindBufferMemory(device_, b.buffer, b.memory, 0);
        if (required & VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT && (r = vk_.vkMapMemory(device_, b.memory, 0, VK_WHOLE_SIZE, 0, &b.mapped)) != VK_SUCCESS)
            return {ErrorCode::DeviceError, vkError("vkMapMemory", r)};
        return {};
    }

    Status makePipeline() {
        VkDescriptorSetLayoutBinding bindings[2] = {
            {0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr},
            {1, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr},
        };
        VkDescriptorSetLayoutCreateInfo lci{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
        lci.bindingCount = 2;
        lci.pBindings = bindings;
        VkResult r = vk_.vkCreateDescriptorSetLayout(device_, &lci, nullptr, &setLayout_);
        if (r != VK_SUCCESS) return {ErrorCode::DeviceError, vkError("vkCreateDescriptorSetLayout", r)};
        VkPipelineLayoutCreateInfo plci{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
        plci.setLayoutCount = 1;
        plci.pSetLayouts = &setLayout_;
        if ((r = vk_.vkCreatePipelineLayout(device_, &plci, nullptr, &pipelineLayout_)) != VK_SUCCESS)
            return {ErrorCode::DeviceError, vkError("vkCreatePipelineLayout", r)};

        VkShaderModuleCreateInfo smci{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
        smci.codeSize = sizeof(kParticlesCompSpv);
        smci.pCode = kParticlesCompSpv;
        VkShaderModule module;
        if ((r = vk_.vkCreateShaderModule(device_, &smci, nullptr, &module)) != VK_SUCCESS)
            return {ErrorCode::DeviceError, vkError("vkCreateShaderModule", r)};
        VkComputePipelineCreateInfo cpci{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
        cpci.stage = {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, nullptr, 0, VK_SHADER_STAGE_COMPUTE_BIT, module, "main", nullptr};
        cpci.layout = pipelineLayout_;
        r = vk_.vkCreateComputePipelines(device_, VK_NULL_HANDLE, 1, &cpci, nullptr, &pipeline_);
        vk_.vkDestroyShaderModule(device_, module, nullptr);
        if (r != VK_SUCCESS) return {ErrorCode::DeviceError, vkError("vkCreateComputePipelines", r)};

        VkDescriptorPoolSize sizes[2] = {{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1}, {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1}};
        VkDescriptorPoolCreateInfo dpci{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        dpci.maxSets = 1;
        dpci.poolSizeCount = 2;
        dpci.pPoolSizes = sizes;
        if ((r = vk_.vkCreateDescriptorPool(device_, &dpci, nullptr, &descriptorPool_)) != VK_SUCCESS)
            return {ErrorCode::DeviceError, vkError("vkCreateDescriptorPool", r)};
        VkDescriptorSetAllocateInfo dsai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        dsai.descriptorPool = descriptorPool_;
        dsai.descriptorSetCount = 1;
        dsai.pSetLayouts = &setLayout_;
        if ((r = vk_.vkAllocateDescriptorSets(device_, &dsai, &set_)) != VK_SUCCESS)
            return {ErrorCode::DeviceError, vkError("vkAllocateDescriptorSets", r)};
        VkDescriptorBufferInfo pinfo{particles_.buffer, 0, VK_WHOLE_SIZE};
        VkDescriptorBufferInfo uinfo{params_.buffer, 0, sizeof(KernelParams)};
        VkWriteDescriptorSet writes[2] = {
            {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr, set_, 0, 0, 1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, nullptr, &pinfo, nullptr},
            {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr, set_, 1, 0, 1, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, nullptr, &uinfo, nullptr},
        };
        vk_.vkUpdateDescriptorSets(device_, 2, writes, 0, nullptr);
        return {};
    }

    void barrier(VkCommandBuffer c, VkAccessFlags src, VkAccessFlags dst, VkPipelineStageFlags srcStage, VkPipelineStageFlags dstStage) {
        VkBufferMemoryBarrier b{VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
        b.srcAccessMask = src;
        b.dstAccessMask = dst;
        b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        b.buffer = particles_.buffer;
        b.size = VK_WHOLE_SIZE;
        vk_.vkCmdPipelineBarrier(c, srcStage, dstStage, 0, 0, nullptr, 1, &b, 0, nullptr);
    }

    // Records, submits and waits. Synchronous for now: simple and safe, but the CPU waits for
    // the GPU each step. Overlapping CPU emission with GPU work is a later optimisation.
    template <typename Record>
    Status submit(Record record) {
        vk_.vkResetCommandBuffer(cmd_, 0);
        VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
        bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        vk_.vkBeginCommandBuffer(cmd_, &bi);
        // Make previous compute writes visible to this submission's transfers and vice versa.
        barrier(cmd_, VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_TRANSFER_WRITE_BIT,
                VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_TRANSFER_READ_BIT | VK_ACCESS_TRANSFER_WRITE_BIT,
                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_TRANSFER_BIT,
                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_TRANSFER_BIT);
        record(cmd_);
        vk_.vkEndCommandBuffer(cmd_);
        VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};
        si.commandBufferCount = 1;
        si.pCommandBuffers = &cmd_;
        VkResult r = vk_.vkQueueSubmit(queue_, 1, &si, fence_);
        if (r != VK_SUCCESS) return {ErrorCode::DeviceError, vkError("vkQueueSubmit", r)};
        r = vk_.vkWaitForFences(device_, 1, &fence_, VK_TRUE, UINT64_MAX);
        vk_.vkResetFences(device_, 1, &fence_);
        if (r != VK_SUCCESS) return {ErrorCode::DeviceError, vkError("vkWaitForFences (device lost?)", r)};
        return {};
    }

    VolkDeviceTable vk_{};
    VkInstance instance_ = VK_NULL_HANDLE;
    VkPhysicalDevice physical_ = VK_NULL_HANDLE;
    VkDevice device_ = VK_NULL_HANDLE;
    VkQueue queue_ = VK_NULL_HANDLE;
    Buf particles_, staging_, params_;
    bool stagingCoherent_ = true;
    VkDescriptorSetLayout setLayout_ = VK_NULL_HANDLE;
    VkPipelineLayout pipelineLayout_ = VK_NULL_HANDLE;
    VkPipeline pipeline_ = VK_NULL_HANDLE;
    VkDescriptorPool descriptorPool_ = VK_NULL_HANDLE;
    VkDescriptorSet set_ = VK_NULL_HANDLE;
    VkCommandPool pool_ = VK_NULL_HANDLE;
    VkCommandBuffer cmd_ = VK_NULL_HANDLE;
    VkFence fence_ = VK_NULL_HANDLE;
    uint32_t slots_ = 0;
};

}  // namespace

std::vector<ComputeDeviceInfo> enumerateVulkanDevices(std::string& reason) {
    std::vector<ComputeDeviceInfo> out;
    if (!loadVulkan(reason)) return out;
    VkInstance instance;
    VkResult r = createInstance(instance);
    if (r != VK_SUCCESS) {
        reason = vkError("vkCreateInstance", r) + " (no Vulkan driver installed?)";
        return out;
    }
    auto devices = computeDevices(instance);
    for (uint32_t i = 0; i < devices.size(); ++i) out.push_back(describe(devices[i].first, i));
    vkDestroyInstance(instance, nullptr);
    if (out.empty()) reason = "Vulkan driver found, but no device supports compute";
    return out;
}

Result<std::unique_ptr<ParticleBackend>> createVulkanParticleBackend(uint32_t gpuIndex, uint32_t slots) {
    using R = Result<std::unique_ptr<ParticleBackend>>;
    std::string reason;
    if (!loadVulkan(reason)) return R::failure(ErrorCode::DeviceUnavailable, reason);
    auto backend = std::make_unique<VulkanParticleBackend>();
    Status s = backend->init(gpuIndex, slots);
    if (!s.ok()) return R::failure(s.error, s.message);
    return R::success(std::move(backend));
}

}  // namespace vexora::physics::detail
