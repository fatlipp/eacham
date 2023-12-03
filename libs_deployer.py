from conan.tools.files import copy
import os


def deploy(graph, output_folder, **kwargs):
    # Note the kwargs argument is mandatory to be robust against future changes.
    for name, dep in graph.root.conanfile.dependencies.items():
        if dep.cpp_info is None or dep.cpp_info.libs is None or len(dep.cpp_info.libdirs) == 0:
            print("libdirs is none: ", name)
        else:
            print("root: ", dep.cpp_info.libdirs)
            copy(graph.root.conanfile, "*", dep.cpp_info.libdirs[0], os.path.join(output_folder, "libs"))