load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "opencv",
    srcs = glob(["lib/*.dylib", "lib/*.so*"]),
    hdrs = glob(["include/opencv4/**/*.hpp", "include/opencv4/**/*.h"]),
    includes = ["include/opencv4"],
    visibility = ["//visibility:public"], 
    linkstatic = 1,
)