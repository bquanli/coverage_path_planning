from conan import ConanFile
from conan.tools.cmake import cmake_layout


class ExampleRecipe(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeDeps", "CMakeToolchain"

    def requirements(self):
        self.requires("yaml-cpp/0.9.0")
        self.requires("spdlog/1.17.0")


    def layout(self):
        cmake_layout(self)