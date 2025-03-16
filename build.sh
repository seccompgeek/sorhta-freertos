#/bin/bash
wget https://developer.arm.com/-/media/Files/downloads/gnu/12.3.rel1/binrel/arm-gnu-toolchain-12.3.rel1-x86_64-aarch64-none-elf.tar.xz
mkdir toolchain 
mv arm-gnu-toolchain-12.3.rel1-x86_64-aarch64-none-elf.tar.xz ./toolchain/.
cd toolchain && tar xvf arm-gnu-toolchain-12.3.rel1-x86_64-aarch64-none-elf.tar.xz
export PATH=$PATH:$PWD/arm-gnu-toolchain-12.3.rel1-x86_64-aarch64-none-elf/bin

