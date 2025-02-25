load("//tools/workspace:github_archive.bzl", "github_archive")

def libcanard_repository(name):
    github_archive(
        name = name,
        repo = "oystub/libcanard",
        #local_override = "../libcanard",
        commit = "6f74bc67656882a4ee51966c7c0022d04fa1a3fb",
        sha256 = "d87e7c6552a0cdf78c2e51bc928a865de5b2039d7549e15ed1dc54955dbefcad",
        build_file = "//tools/workspace/libcanard:BUILD.inject",
    )
