---
source_type: rendered_autoware_docs
site: autoware_universe
title: Autoware Behavior Velocity Module Template
canonical_url: https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_velocity_planner/autoware_behavior_velocity_template_module/
rendered_html: rendered/autoware_universe_html/index-214.html
generated: 2026-06-24T14:59:35.533488+00:00
---

> **Autoware License Notice**
> This file is part of a local planning-reference bundle derived from, summarizing, or reorganizing materials from Autoware Foundation repositories, including Autoware Universe. The upstream Autoware materials are distributed under the Apache License, Version 2.0.
>
> This local copy is kept for AjouNice2026 planning-reference and ROS 1 porting study purposes. Local changes may include pruning, reorganization, format conversion, extracted tables, and generated summaries. This notice does not change the upstream license terms.
>
> See `LICENSE_AUTOWARE_APACHE_2_0.md`, `NOTICE.md`, and `meta/source_repositories.md` in this bundle for license text, attribution notices, and source/version provenance.

## Template

A template for behavior velocity modules based on the autoware\_behavior\_velocity\_speed\_bump\_module.

# Autoware Behavior Velocity Module Template

## `Scene`

### `TemplateModule` Class

The `TemplateModule` class serves as a foundation for creating a scene module within the Autoware behavior velocity planner. It defines the core methods and functionality needed for the module's behavior. You should replace the placeholder code with actual implementations tailored to your specific behavior velocity module.

#### Constructor

- The constructor for `TemplateModule` takes the essential parameters to create a module: `const int64_t module_id`, `const rclcpp::Logger & logger`, and `const rclcpp::Clock::SharedPtr clock`. These parameters are supplied by the `TemplateModuleManager` when registering a new module. Other parameters can be added to the constructor, if required by your specific module implementation.

#### `modifyPathVelocity` Method

- This method, defined in the `TemplateModule` class, is expected to modify the velocity of the input path based on certain conditions. In the provided code, it logs an informational message once when the template module is executing.
- The specific logic for velocity modification should be implemented in this method based on the module's requirements.

#### `createDebugMarkerArray` Method

- This method, also defined in the `TemplateModule` class, is responsible for creating a visualization of debug markers and returning them as a `visualization_msgs::msg::MarkerArray`. In the provided code, it returns an empty `MarkerArray`.
- You should implement the logic to generate debug markers specific to your module's functionality.

#### `createVirtualWalls` Method

- The `createVirtualWalls` method creates virtual walls for the scene and returns them as `autoware::motion_utils::VirtualWalls`. In the provided code, it returns an empty `VirtualWalls` object.
- You should implement the logic to create virtual walls based on your module's requirements.

## `Manager`

The managing of your modules is defined in manager.hpp and manager.cpp. The managing is handled by two classes:

- The `TemplateModuleManager` class defines the core logic for managing and launching the behavior\_velocity\_template scenes (defined in behavior\_velocity\_template\_module/src/scene.cpp/hpp). It inherits essential manager attributes from its parent class `SceneModuleManagerInterface`.
- The `TemplateModulePlugin` class provides a way to integrate the `TemplateModuleManager` into the logic of the Behavior Velocity Planner.

### `TemplateModuleManager` Class

#### Constructor `TemplateModuleManager`

- This is the constructor of the `TemplateModuleManager` class, and it takes an `rclcpp::Node` reference as a parameter.
- It initializes a member variable `dummy_parameter_` to 0.0.

#### `getModuleName()` Method

- This method is an override of a virtual method from the `SceneModuleManagerInterface` class.
- It returns a pointer to a constant character string, which is the name of the module. In this case, it returns "template" as the module name.

#### `launchNewModules()` Method

- This is a private method that takes an argument of type `autoware_internal_planning_msgs::msg::PathWithLaneId`.
- It is responsible for launching new modules based on the provided path information (PathWithLaneId). The implementation of this method involves initializing and configuring modules specific to your behavior velocity planner by using the `TemplateModule` class.
- In the provided source code, it initializes a `module_id` to 0 and checks if a module with the same ID is already registered. If not, it registers a new `TemplateModule` with the module ID. Note that each module managed by the `TemplateModuleManager` should have a unique ID. The template code registers a single module, so the `module_id` is set as 0 for simplicity.

#### `getModuleExpiredFunction()` Method

- This is a private method that takes an argument of type `autoware_internal_planning_msgs::msg::PathWithLaneId`.
- It returns a `std::function<bool(const std::shared_ptr<SceneModuleInterface>&)>`. This function is used by the behavior velocity planner to determine whether a particular module has expired or not based on the given path.
- The implementation of this method is expected to return a function that can be used to check the expiration status of modules.

Please note that the specific functionality of the methods `launchNewModules()` and `getModuleExpiredFunction()` would depend on the details of your behavior velocity modules and how they are intended to be managed within the Autoware system. You would need to implement these methods according to your module's requirements.

### `TemplateModulePlugin` Class

#### `TemplateModulePlugin` Class

- This class inherits from `PluginWrapper<TemplateModuleManager>`. It essentially wraps your `TemplateModuleManager` class within a plugin, which can be loaded and managed dynamically.

## `Example Usage`

In the following example, we take each point of the path, and multiply it by 2. Essentially duplicating the speed. Note that the velocity smoother will further modify the path speed after all the behavior velocity modules are executed.

```cpp
bool TemplateModule::modifyPathVelocity(
[[maybe\_unused]] PathWithLaneId \* path, [[maybe\_unused]] StopReason \* stop\_reason)
{
for (auto & p : path->points) {
p.point.longitudinal\_velocity\_mps \*= 2.0;
}

return false;
}
```
