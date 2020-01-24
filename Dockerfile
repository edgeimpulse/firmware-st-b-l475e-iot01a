FROM python:3.7.5-stretch

WORKDIR /app

# APT packages
RUN apt update && apt install -y zip

RUN cd .. && \
    wget https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/9-2019q4/RC2.1/gcc-arm-none-eabi-9-2019-q4-major-x86_64-linux.tar.bz2 && \
    tar xjf gcc-arm-none-eabi-9-2019-q4-major-x86_64-linux.tar.bz2 && \
    echo "PATH=$PATH:/gcc-arm-none-eabi-9-2019-q4-major/bin" >> ~/.bashrc && \
    cd /app

# Python dependencies
COPY requirements.txt ./
RUN pip3 --no-cache-dir install -r requirements.txt

RUN apt install -y mercurial

# Project dependencies
COPY *.lib ./
COPY source/edge-impulse-sdk/*.lib ./source/edge-impulse-sdk/
RUN rm -rf mbed-os/ && echo 'ROOT=.' > .mbed && mbed deploy

# Now copy the real project (.dockerignore has all the libraries)
COPY . ./

# create and build
RUN mbed config -G GCC_ARM_PATH /gcc-arm-none-eabi-9-2019-q4-major/bin

RUN mbed compile -t GCC_ARM -m DISCO_L475VG_IOT01A --profile=debug && \
    rm -f *.zip && \
    zip -0 -q -r mbed-firmware.zip . -x *.git*

CMD cp /app/mbed-firmware.zip /home/mbed-firmware.zip
