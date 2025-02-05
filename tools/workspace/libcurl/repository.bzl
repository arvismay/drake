# -*- mode: python -*-

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def libcurl_repository(
        name,
        licenses = ["notice"],  # curl
        modname = "libcurl",
        pkg_config_paths = [],
        homebrew_subdir = "opt/curl/lib/pkgconfig",
        **kwargs):
    pkg_config_repository(
        name = name,
        licenses = licenses,
        modname = modname,
        pkg_config_paths = pkg_config_paths,
        **kwargs
    )
