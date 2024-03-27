#!/bin/bash

CC="g++"
S_CFLAGS="-std=c++11 -Wall -Wextra -Wshadow -fsanitize=address,undefined -fno-exceptions -fno-rtti"
F_CFLAGS="-std=c++11 -Wno-unused-result -fno-exceptions -fno-rtti -Ofast"
TARGET="./src/main.cpp"

wrong_usage() {
    printf "\033[93mUsage:\033[0m %s <compilation option> <scene option>\n\n" "$0"
    printf "\033[93mCompilation options:\033[0m\n"
    printf "\033[95m[SAFE]\033[0m -s\n"
    printf "\033[95m[FAST]\033[0m -f\n\n"
    printf "\033[93mScene options:\033[0m\n"
    printf "\033[95m[SAMPLE]\033[0m    -s\n"
    printf "\033[95m[CORNELL]\033[0m   -c\n"
    printf "\033[95m[VAPORWAVE]\033[0m -v\n\n"
    printf "You can also set the sdl file path as the scene option\n"
    exit 1
}

if [ $# -ne 2 ]; then
    wrong_usage
fi

COMPILATION_OPTION=$1
if [ "$COMPILATION_OPTION" != "-s" ] && [ "$COMPILATION_OPTION" != "-f" ]; then
    wrong_usage
fi

SCENE_OPTION=$2

printf "Compiling '%s' with C++11...\n" "$TARGET"

$CC $(if [ "${COMPILATION_OPTION:1:1}" == "s" ]; then printf "%s" "$S_CFLAGS"; else printf "%s" "$F_CFLAGS"; fi) $TARGET

if [ $? -eq 0 ]; then
    printf "Done.\n\n"
else
    printf "\033[91mFailed.\033[0m\n"
    exit 1
fi

printf "Rendering image...\n"

./a.out $SCENE_OPTION;

if [ $? -eq 0 ]; then
    printf "Done.\n\n"
else
    printf "\033[91mFailed.\033[0m\n"
fi

rm a.out

if [ $? -eq 0 ]; then
    printf "\033[93mNote:\033[0m The generated image is located in the './img' directory.\n"
fi
