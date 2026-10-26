# cmake -DCHIPS=<src/kvasir/Devices/I2C/chips> -P catalogue_lists.cmake (ctest i2c_catalogue_lists)
#
# The two hand-kept lists of the chip descriptions agree with the directory: chips/All.hpp includes every header there
# (and none that is not), and chips/Every.hpp - the list identify_test and oracle_test walk - names at least one
# description each header defines (a `struct` or `using` at namespace scope). A header that holds no description to walk
# is NOT_WALKED, with its reason.
set(NOT_WALKED GeneralCall.hpp) # the general call: a write to address 0 on demand, nothing that answers an identity

file(
    GLOB headers
    RELATIVE ${CHIPS}
    ${CHIPS}/*.hpp)
list(REMOVE_ITEM headers All.hpp Every.hpp)
list(LENGTH headers count)
if(count EQUAL 0)
    message(FATAL_ERROR "no chip header found in ${CHIPS}")
endif()

file(READ ${CHIPS}/All.hpp all)
string(REGEX MATCHALL "#include \"[A-Za-z0-9_]+\\.hpp\"" includes "${all}")
set(included)
foreach(line IN LISTS includes)
    string(REGEX REPLACE "#include \"([A-Za-z0-9_]+\\.hpp)\"" "\\1" name "${line}")
    list(APPEND included ${name})
endforeach()

file(READ ${CHIPS}/Every.hpp every)
string(REGEX MATCHALL "Chips::[A-Za-z0-9_]+" listed "${every}")

set(problems)
foreach(header IN LISTS headers)
    if(NOT header IN_LIST included)
        list(APPEND problems "chips/All.hpp does not include ${header}")
    endif()
    if(header IN_LIST NOT_WALKED)
        continue()
    endif()
    file(STRINGS ${CHIPS}/${header} decls REGEX "^(struct|class|using) [A-Za-z0-9_]+")
    set(found FALSE)
    foreach(decl IN LISTS decls)
        string(REGEX REPLACE "^(struct|class|using) ([A-Za-z0-9_]+).*" "\\2" name "${decl}")
        if("Chips::${name}" IN_LIST listed)
            set(found TRUE)
            break()
        endif()
    endforeach()
    if(NOT found)
        list(APPEND problems
             "chips/Every.hpp names no description of ${header} (or add it to NOT_WALKED with a reason)")
    endif()
endforeach()
foreach(name IN LISTS included)
    if(NOT name IN_LIST headers)
        list(APPEND problems "chips/All.hpp includes ${name}, which is not in chips/")
    endif()
endforeach()
foreach(name IN LISTS NOT_WALKED)
    if(NOT name IN_LIST headers)
        list(APPEND problems "NOT_WALKED names ${name}, which is not in chips/")
    endif()
endforeach()

if(problems)
    list(JOIN problems "\n  " text)
    message(FATAL_ERROR "the chip lists disagree with chips/:\n  ${text}")
endif()
message(STATUS "chips/All.hpp and chips/Every.hpp cover all ${count} chip headers")
