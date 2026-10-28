add_executable(vexora_sim main.cpp Session.cpp Schema.cpp)
target_link_libraries(vexora_sim PRIVATE vexora::physics nlohmann_json::nlohmann_json)
if(MSVC)
  target_compile_options(vexora_sim PRIVATE /W4 /utf-8)  # Thai UI strings in source are UTF-8.
else()
  target_compile_options(vexora_sim PRIVATE -Wall -Wextra)
endif()
