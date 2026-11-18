# Shaders are Vulkan-style GLSL 440 in shaders/, baked by qsb into .qsb files that
# carry SPIR-V, HLSL (D3D11/12), MSL (Metal) and GLSL (OpenGL) variants.
# qsb is a build tool (GPLv3); only QtGui's QShader loads the result at run time.
function(corv_add_shaders target prefix base)
    qt6_add_shaders(${target} "${target}_shaders"
        PREFIX "${prefix}"
        BASE "${base}"
        BATCHABLE
        PRECOMPILE
        GLSL "300 es,330"
        HLSL 50
        MSL 12
        FILES ${ARGN})
endfunction()
