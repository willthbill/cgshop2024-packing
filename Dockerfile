# Use the latest Arch Linux base image
FROM archlinux

WORKDIR /app

# Update the package database and upgrade the system
RUN pacman -Syu --noconfirm

# Install Python 3.10, pip, CGAL, and CMake
RUN pacman -S --noconfirm cgal cmake git gcc boost make scons fzf direnv which tree vim vi neovim htop tk fontconfig ttf-dejavu

# Production
#ARG UPDATE_LIB=false
#ARG GITHUB_AUTH=notoken
#RUN git clone https://${GITHUB_AUTH}@github.com/willthbill/cgshop-lib.git
#WORKDIR /app/cgshop-lib
#RUN ./scripts/configure_debug
#RUN ./scripts/build
#WORKDIR /app

WORKDIR /app
RUN python -m venv venv
ENV PATH="venv/bin:$PATH"
RUN pip install --no-cache-dir matplotlib numpy pybind11

COPY .bashrc /root/

# Set the default command for the container
WORKDIR /app/max-polygon-packing
CMD ["/bin/bash", "-c", "direnv allow; bash"]

