load("//tools/workspace:github_archive.bzl", "github_archive")

def libcanard_repository(name):
    github_archive(
        name = name,
        repo = "dronecan/libcanard",
        commit = "5d7b725ce114d079588cd9a6cabc73e333ea1cff",
        sha256 = "5d86e76dea8563d3a4523404d33e0694d712cd2c1228fb1c4fe8908544daf065",
        build_file = "//tools/workspace/libcanard:BUILD.inject",
    )
