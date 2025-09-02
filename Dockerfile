FROM ubuntu:22.04

# Set noninteractive mode and timezone
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# ========== Install Build Dependencies ==========
RUN apt-get update && apt-get install -y \
    build-essential cmake git wget curl ca-certificates \
    python3 python3-pip jq \
    && rm -rf /var/lib/apt/lists/*

# ========== Create Non-root User ==========
RUN useradd -ms /bin/bash osk
WORKDIR /home/osk

# ========== Build NASA cFS ==========
RUN git clone --recurse-submodules https://github.com/nasa/cFS.git
WORKDIR /home/osk/cFS
RUN cp cfe/cmake/Makefile.sample Makefile && \
    cp -r cfe/cmake/sample_defs . && \
    make prep && make && make install

# ========== Build 42 (Headless Mode) ==========
WORKDIR /home/osk
RUN git clone https://github.com/ericstoneking/42.git
WORKDIR /home/osk/42
RUN sed -i 's/-D _ENABLE_GUI_//g' Makefile && \
    sed -i 's/-D _USE_GLUT_//g' Makefile && \
    sed -i 's/-D _USE_SHADERS_//g' Makefile && \
    sed -i 's/-I \/usr\/include\/GL\///g' Makefile && \
    sed -i 's/-lglut -lGLU -lGL -lXmu -lXi -lX11//g' Makefile && \
    sed -i '/Object\/42gl\.o/d' Makefile && \
    make && \
    chown -R osk:osk /home/osk/42

# ========== Create Minimal Configs for 42 ==========
RUN mkdir -p /home/osk/42/InOut && \
    echo '****************** 42: Spacecraft Simulator ******************' > /home/osk/42/InOut/Sim.txt && \
    echo 'FALSE                       ! Echo to stdout' >> /home/osk/42/InOut/Sim.txt && \
    echo 'TRUE                        ! Quiet' >> /home/osk/42/InOut/Sim.txt && \
    echo 'FALSE                       ! Graphics Enable' >> /home/osk/42/InOut/Sim.txt && \
    echo '0.1                         ! Sim Step Size (sec)' >> /home/osk/42/InOut/Sim.txt && \
    echo 'DURATION_PLACEHOLDER        ! Duration (will be replaced)' >> /home/osk/42/InOut/Sim.txt && \
    echo '2000 01 01 12 00 00.0       ! Date and Time (UTC)' >> /home/osk/42/InOut/Sim.txt && \
    echo '1                           ! Number of Spacecraft' >> /home/osk/42/InOut/Sim.txt && \
    echo 'SC_Simple.txt               ! Spacecraft Description File' >> /home/osk/42/InOut/Sim.txt && \
    echo 'Orb_LEO.txt                 ! Orbit Description File' >> /home/osk/42/InOut/Sim.txt && \
    echo 'NONE                        ! Graphics Input File' >> /home/osk/42/InOut/Sim.txt && \
    echo 'NONE                        ! Command Script File' >> /home/osk/42/InOut/Sim.txt

# ========== Fault Injection Setup ==========
ARG MOCK_FAULT_URL="https://mockapi.example.com/api/get_faults"
ARG TEAM_ID="default_team"
RUN mkdir -p /home/osk/extensions && \
    echo "{\"team_id\":\"${TEAM_ID}\",\"request_type\":\"code_faults\"}" > /home/osk/payload.json && \
    (curl -s -X POST "${MOCK_FAULT_URL}" \
        -H "Content-Type: application/json" \
        --data-binary @/home/osk/payload.json \
        -o /home/osk/extensions/fault_scripts.tar.gz || \
        echo "# Mock fault injection" > /home/osk/extensions/fault_mock.py) && \
    rm -f /home/osk/payload.json

# ========== Copy ADCS C++ Source Files ==========
COPY adcs_controller.cpp /home/osk/
COPY microcontroller.cpp /home/osk/
COPY pid_controller.cpp /home/osk/

# ========== Compile ADCS Simulation ==========
RUN g++ -DSIM_DURATION=3600 -DALTITUDE_THRESHOLD=10.0 \
    -o /home/osk/adcs_sim \
    /home/osk/adcs_controller.cpp /home/osk/microcontroller.cpp /home/osk/pid_controller.cpp

# ========== Add Telemetry Bridge and Startup Script ==========
COPY telemetry_bridge.py /home/osk/telemetry_bridge.py
COPY start_all.sh /home/osk/start_all.sh

# ✅ Fix ownership and permissions BEFORE switching user
RUN chown osk:osk /home/osk/start_all.sh /home/osk/telemetry_bridge.py && \
    chmod +x /home/osk/start_all.sh /home/osk/telemetry_bridge.py && \
    mkdir -p /etc/cf && chown osk:osk /etc/cf

# ✅ Switch to non-root user
USER osk

# ========== Set Environment Variables ==========
ENV SIM_TIME=3600
ENV ALTITUDE_THRESHOLD=10.0
ENV TEAM_ID=default

# ========== Create Log Directory ==========
RUN mkdir -p /tmp

# ========== Expose Ports ==========
EXPOSE 1234/udp 1235/udp 5000/tcp

# ========== Final Setup ==========
WORKDIR /home/osk
CMD [ "/bin/bash" ]
# CMD ["/home/osk/start_all.sh"]
