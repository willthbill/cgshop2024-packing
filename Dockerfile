# Use the latest Arch Linux base image
# FROM archlinux
FROM greyltc/archlinux-aur:yay


# Install Python 3.10, pip, CGAL, and CMake
RUN pacman -Syu --noconfirm && pacman -S --noconfirm cgal cmake git gcc boost make scons fzf direnv which tree vim vi neovim htop tk fontconfig ttf-dejavu fd scip tmux gdb ripgrep

# Install aur packages
RUN pacman -Syu --noconfirm && aur-install gurobi

RUN pacman -Syu --noconfirm sudo

# Production
#ARG UPDATE_LIB=false
#ARG GITHUB_AUTH=notoken
#RUN git clone https://${GITHUB_AUTH}@github.com/willthbill/cgshop-lib.git
#WORKDIR /app/cgshop-lib
#RUN ./scripts/configure_debug
#RUN ./scripts/build
#WORKDIR /app

RUN groupadd -g 1000 mygroup && \
    useradd -m -u 1000 -g 1000 -s /bin/bash myuser
RUN usermod -aG wheel myuser
RUN sed -i 's/# %wheel ALL=(ALL:ALL) NOPASSWD: ALL/%wheel ALL=(ALL:ALL) NOPASSWD: ALL/' /etc/sudoers
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN cat /etc/sudoers

WORKDIR /app
RUN chown -R myuser:mygroup /app
USER myuser
RUN python -m venv venv
ENV PATH="venv/bin:$PATH"
RUN pip install -U -v cgshop2024-pyutils
RUN pip install --no-cache-dir matplotlib numpy pybind11

COPY .bashrc /home/myuser/

RUN echo 'source /home/myuser/.bashrc' >> ~/.profile
RUN echo 'source /home/myuser/.bashrc' >> ~/.bash_profile

# Set the default command for the container
WORKDIR /app/max-polygon-packing
RUN sudo mkdir -p /opt/gurobi

# quick fix
RUN sudo ln -sf /usr/lib/libgurobi_g++8.5.a /usr/lib/libgurobi_c++.a

CMD ["/bin/bash", "-c", "sudo cp /app/max-polygon-packing/licenses/gurobi.lic /opt/gurobi; direnv allow; bash"]

