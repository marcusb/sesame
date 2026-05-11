add_subdirectory(berry)
add_subdirectory(berry_matter)

# coc scans the build-tree solidify dirs produced by the berry_matter
# solidify targets, so it must run after the .be sources have been compiled.
add_dependencies(berry_coc berry_matter_solidify berry_matter_solidify_extra)
