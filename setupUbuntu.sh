#!/bin/bash
############################## General SetUp ##############################
#Create General Directories
echo -e "\033[7;36m ######## Creating General Directories  ########\033[m"
#Create Code Directories
FILE=~/Code
if [[ ! -d "$FILE" ]]
then
  echo -e "\033[5;36m ---> Creating: ~/Code \033[m"
  mkdir ~/Code/
fi

#echo -e "\033[5;36m ---> Installing: gnome-terminal  \033[m"
#sudo apt install gnome-terminal

echo -e "\033[5;36m ---> Installing: adwaita (WSL)  \033[m"
sudo apt install adwaita-icon-theme-full
echo -e "\033[5;36m ---> Installing: libvtk7  \033[m"
sudo apt-get install libvtk7.1

echo -e "\033[5;36m ---> Installing: boost  \033[m"
sudo apt install libboost-all-dev

echo -e "\033[5;36m ---> Installing: libparmetis.so \033[m"
sudo apt-get install -y libparmetis-dev


echo -e "\033[5;36m ---> Installing: Terminator (no ther James Cameron movie)\033[m"
sudo apt-get install terminator

############################## CERES solver ##############################
echo -e "\033[7;36m ######## CERES Solver (Stable)  ########\033[m"
echo -e "\033[1;36m Install Ceres Solver: Yes (1) / No (0) \033[m"
read ceres_flag
if [[ $ceres_flag -eq 1 ]]
then
    echo -e "\033[5;36m ---> Installing dependencies \033[m"
    sudo apt-get install cmake
    sudo apt-get install libgoogle-glog-dev libgflags-dev
    sudo apt-get install libatlas-base-dev
    sudo apt-get install libeigen3-dev
    sudo apt-get install libsuitesparse-dev

    echo -e "\033[5;36m ---> Downloading Ceres\033[m"
    cd ~/Code/
    mkdir CeresSolver
    cd ./CeresSolver
    wget http://ceres-solver.org/ceres-solver-2.0.0.tar.gz 

    echo -e "\033[5;36m ---> Installing \033[m"
    tar zxf ceres-solver-2.0.0.tar.gz
    mkdir ceres-bin
    cd ceres-bin
    cmake ../ceres-solver-2.0.0
    make -j8
    make -j8 test
    sudo make -j8 install
else
    echo -e "\033[1;35m ---> Ceres Solver installation skipped \033[m"
fi

############################## Python Utilities ##############################
echo -e "\033[7;36m ######## Python utilities  ########\033[m"
echo -e "\033[1;36m Install python_utilities: Yes (1) / No (0) \033[m"
read python_flag
if [[ $python_flag -eq 1 ]]
then
    echo -e "\033[5;36m ---> Installing: pip (3.6) \033[m"
    sudo apt install python3-pip
    echo -e "\033[5;36m ---> Installing: catkin_pkg  (3.6) \033[m"
    sudo pip3 install catkin_pkg
    echo -e "\033[5;36m ---> Installing: empy(3.6) \033[m"
    sudo apt-get install python3-empy
else
    echo -e "\033[1;35m ---> Python_utilities installation skipped \033[m"
fi


############################## GTSAM 4.0.2 ##############################
echo -e "\033[7;36m ######## GTSAM 4.0.2  ########\033[m"
echo -e "\033[1;36m Install GTSAM: Yes (1) / No (0) \033[m"
read gtsam_flag
if [[ $gtsam_flag -eq 1 ]]
then
  wget -O ~/Code/gtsam.zip https://github.com/borglab/gtsam/archive/4.0.2.zip
  cd ~/Code/ && unzip gtsam.zip -d ~/Code/
  cd ~/Code/gtsam-4.0.2/
  mkdir build && cd build
  cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DCMAKE_BUILD_TYPE=Release ..
  sudo make install -j8
  cd ~/Code/
  rm gtsam.zip
  
else
  echo -e "\033[1;35m ---> GTSAM installation skipped \033[m"
fi

############################## VIM ##############################
echo -e "\033[7;36m ######## Vi IMproved ########\033[m"
echo -e "\033[1;36m Install VIM: Yes (1) / No (0) \033[m"
read vim_flag
if [[ $vim_flag -eq 1 ]]
then
  #Update system
  echo -e "\033[5;36m ---> Updating System \033[m"
  sudo apt-get install cmake
  sudo apt update

  #Install Dependencies 
  echo -e "\033[5;36m ---> Installing Dependencies\033[m"
  sudo apt install libncurses5-dev libgtk2.0-dev libatk1.0-dev \
      libcairo2-dev libx11-dev libxpm-dev libxt-dev python2-dev \
      python3-dev ruby-dev lua5.2 liblua5.2-dev libperl-dev git
  sudo apt-get build-dep vim

  echo -e "\033[5;36m ---> Remove previous versions\033[m"
  sudo apt remove vim vim-runtime gvim

  echo -e "\033[5;36m ---> Install VIM\033[m"
  cd ~/Code
  git clone https://github.com/vim/vim.git
  cd vim
  ./configure --with-features=huge \
            --enable-multibyte \
            --enable-rubyinterp=yes \
            --enable-python3interp=yes \
            --with-python3-config-dir=$(python3-config --configdir) \
            --enable-perlinterp=yes \
            --enable-luainterp=yes \
            --enable-gui=gtk2 \
            --enable-cscope \
            --prefix=/usr/local
  make VIMRUNTIMEDIR=/usr/local/share/vim/vim82
  sudo apt install checkinstall
  cd ~/Code/vim
  sudo make install

  sudo update-alternatives --install /usr/bin/editor editor /usr/local/bin/vim 1
  sudo update-alternatives --set editor /usr/local/bin/vim
  sudo update-alternatives --install /usr/bin/vi vi /usr/local/bin/vim 1
  sudo update-alternatives --set vi /usr/local/bin/vim


  #Verify VIM
  echo -e "\033[5;36m ---> Verify VIM\033[m"
  vim --version

  echo -e "\033[5;36m ---> Install ctags (Source File Outliner)\033[m"
  sudo apt-get install ctags
  echo -e "\033[5;36m ---> Install ack (Search Asynchronously) \033[m"
  sudo apt-get install ack 
  #echo -e "\033[5;36m ---> Install xbuild (YouCompleteMe) \033[m"
  #sudo apt install build-essential cmake vim-nox python3-dev
  #sudo apt install mono-complete golang nodejs default-jdk npm

  #Configuring VIM
  echo -e "\033[5;36m ---> Configuring \033[m"
  echo -e "\033[5;36m \t ---> Installing Bundle \033[m"
  vim --version

  cd ~
  file=./.vim
  if [[ ! -d "$file" ]]
  then
    mkdir ./.vim
  fi
  cd ./.vim
  file=./bundle
  if [[ ! -d "$file" ]]
  then
    mkdir ./bundle
  fi
  cd ./bundle
  git clone https://github.com/VundleVim/Vundle.vim.git ~/.vim/bundle/Vundle.vim
  
  cd ~
  # Settings and plugins from:
  # https://dane-bulat.medium.com/how-to-turn-vim-into-a-lightweight-ide-6185e0f47b79

  echo "\" =============== Vundle ================= " >> ~/.vimrc 
  echo "set nocompatible" >> ~/.vimrc
  echo "filetype off" >> ~/.vimrc
  echo "set rtp+=~/.vim/bundle/Vundle.vim" >> ~/.vimrc
  echo "call vundle#begin()" >> ~/.vimrc

  echo "\" =============== Plugins ================= " >> ~/.vimrc 

  echo "Plugin 'VundleVim/Vundle.vim'" >> ~/.vimrc
  echo "Plugin 'tpope/vim-fugitive'" >> ~/.vimrc
  echo "Plugin 'git://git.wincent.com/command-t.git'" >> ~/.vimrc
  echo "Plugin 'rstacruz/sparkup', {'rtp': 'vim/'}" >> ~/.vimrc

  echo "Plugin 'sheerun/vim-polyglot'" >> ~/.vimrc
  echo "Plugin 'jiangmiao/auto-pairs'" >> ~/.vimrc
  echo "Plugin 'preservim/nerdtree'" >> ~/.vimrc
  echo "Plugin 'preservim/tagbar'" >> ~/.vimrc
  echo "Plugin 'dyng/ctrlsf.vim'" >> ~/.vimrc
  echo "Plugin 'dense-analysis/ale'" >> ~/.vimrc
  #echo "Plugin 'LaTeX-Suite-aka-Vim-LaTeX'" >> ~/.vimrc
  #echo "Plugin 'Valloric/YouCompleteMe', { 'commit':'d98f896' }" >> ~/.vimrc
  #echo "Plugin 'Valloric/YouCompleteMe'" >> ~/.vimrc
  
  echo "call vundle#end()" >> ~/.vimrc
  echo "filetype plugin indent on " >> ~/.vimrc
  echo "\" =============== General setup ================= " >> ~/.vimrc 
  echo "set nu                 " >> ~/.vimrc 
  echo "syntax on              " >> ~/.vimrc 
  echo "set incsearch          " >> ~/.vimrc 
  echo "set hlsearch           " >> ~/.vimrc 
  echo "set splitbelow         " >> ~/.vimrc 
  echo "set mouse=c            " >> ~/.vimrc 
  echo "set tabstop=4          " >> ~/.vimrc 
  echo "set shiftwidth=4       " >> ~/.vimrc 
  echo "set expandtab          " >> ~/.vimrc  
  echo "set ruler              " >> ~/.vimrc
  echo "set ignorecase         " >> ~/.vimrc
  echo "set smartcase          " >> ~/.vimrc
  echo "set incsearch          " >> ~/.vimrc
  echo "set lazyredraw         " >> ~/.vimrc
  echo "set magic              " >> ~/.vimrc
  echo "set showmatch          " >> ~/.vimrc
  echo "set noerrorbells       " >> ~/.vimrc
  echo "set novisualbell       " >> ~/.vimrc
  echo "set t_vb=              " >> ~/.vimrc
  echo "set tm=500             " >> ~/.vimrc
  echo "set smarttab           " >> ~/.vimrc
  echo "set ai \"Auto indent   " >> ~/.vimrc
  echo "set si \"Smart indent  " >> ~/.vimrc
  echo "set wrap \"Wrap lines  " >> ~/.vimrc
  echo "set laststatus=2       " >> ~/.vimrc
  

  echo "map <F3> :call CompileRun()<CR>               " >> ~/.vimrc
  echo "imap <F3> <Esc>:call CompileRun()<CR>               " >> ~/.vimrc
  echo "vmap <F3> <Esc>:call CompileRun()<CR>               " >> ~/.vimrc

  echo "func! CompileRun()               " >> ~/.vimrc
  echo "exec \"w\"                       " >> ~/.vimrc
  echo "if &filetype == 'c'              " >> ~/.vimrc
  echo "    exec \"!gcc % -o %<\"        " >> ~/.vimrc
  echo "    exec \"!time ./%<\"          " >> ~/.vimrc
  echo "elseif &filetype == 'cpp'        " >> ~/.vimrc
  echo "    exec \"!g++ % -o %<\"        " >> ~/.vimrc
  echo "    exec \"!time ./%<\"          " >> ~/.vimrc
  echo "elseif &filetype == 'java'       " >> ~/.vimrc
  echo "    exec \"!javac %\"            " >> ~/.vimrc
  echo "    exec \"!time java %\"        " >> ~/.vimrc
  echo "elseif &filetype == 'sh'         " >> ~/.vimrc
  echo "    exec \"!time bash %\"        " >> ~/.vimrc
  echo "elseif &filetype == 'python'     " >> ~/.vimrc
  echo "    exec \"!time python3 %\"     " >> ~/.vimrc
  echo "elseif &filetype == 'html'       " >> ~/.vimrc
  echo "    exec \"!google-chrome % &\"  " >> ~/.vimrc
  echo "elseif &filetype == 'go'         " >> ~/.vimrc
  echo "    exec \"!go build %<\"        " >> ~/.vimrc
  echo "    exec \"!time go run %\"      " >> ~/.vimrc
  echo "elseif &filetype == 'matlab'     " >> ~/.vimrc
  echo "    exec \"!time octave %\"      " >> ~/.vimrc
  echo "elseif &filetype == 'tex'        " >> ~/.vimrc
  echo "    exec \"!pdflatex %\"         " >> ~/.vimrc
  echo "endif                            " >> ~/.vimrc
  echo "endfunc                          " >> ~/.vimrc

  echo "autocmd BufReadPost *                                                       " >> ~/.vimrc
  echo "    \\ if line(\"'\\\"\") > 0 && line(\"'\\\"\") <= line(\"$\") |           " >> ~/.vimrc
  echo "    \\   exe \"normal! g\`\\\"\" |                                            " >> ~/.vimrc
  echo "    \\ endif                                                               " >> ~/.vimrc


  echo "\" =============== Auto Pair setup  ================= " >> ~/.vimrc 
  echo "let g:AutoPairsShortcutToggle = '<C-P>' " >> ~/.vimrc  

  echo "\" =============== Nerd tree setup ================= " >> ~/.vimrc 
  echo "let NERDTreeShowBookmarks = 1   " >>~/.vimrc
  echo "let NERDTreeShowHidden = 1" >>~/.vimrc
  echo "let NERDTreeShowLineNumbers = 0" >>~/.vimrc
  echo "let NERDTreeMinimalMenu = 1" >>~/.vimrc
  echo "let NERDTreeWinPos = \"left\" " >>~/.vimrc
  echo "let NERDTreeWinSize = 31" >>~/.vimrc
  echo "nnoremap <leader>n :NERDTreeFocus<CR>     ">>~/.vimrc
  echo "nnoremap <C-n> :NERDTree<CR>              ">>~/.vimrc
  echo "nnoremap <C-t> :NERDTreeToggle<CR>        ">>~/.vimrc
  echo "nnoremap <C-f> :NERDTreeFind<CR>          ">>~/.vimrc

  #echo "map <leader>nn :NERDTreeToggle<cr> ">>~/.vimrc
  #echo "map <leader>nb :NERDTreeFromBookmark<Space> ">>~/.vimrc
  #echo "map <leader>nf :NERDTreeFind<cr> ">>~/.vimrc
  echo "" >>~/.vimrc

  echo "\" =============== Tagbar setup ================= " >> ~/.vimrc 
  echo "let g:tagbar_autofocus = 1 " >> ~/.vimrc  
  echo "let g:tagbar_autoshowtag = 1" >> ~/.vimrc  
  echo "let g:tagbar_position = 'botright vertical'" >> ~/.vimrc  
  echo "nmap <F8> :TagbarToggle<CR>" >> ~/.vimrc  

  echo "\" =============== CtrlFS setup ================= " >> ~/.vimrc 
  echo "let g:ctrlsf_backend = 'ack'" >> ~/.vimrc  
  echo "let g:ctrlsf_auto_close = { \"normal\":0, \"compact\":0 }" >> ~/.vimrc  
  echo "let g:ctrlsf_auto_focus = { \"at\":\"start\" }" >> ~/.vimrc  
  echo "let g:ctrlsf_auto_preview = 0" >> ~/.vimrc  
  echo "let g:ctrlsf_case_sensitive = 'smart'" >> ~/.vimrc  
  echo "let g:ctrlsf_default_view = 'normal'" >> ~/.vimrc  
  echo "let g:ctrlsf_regex_pattern = 0" >> ~/.vimrc  
  echo "let g:ctrlsf_position = 'right'" >> ~/.vimrc  
  echo "let g:ctrlsf_winsize = '46'" >> ~/.vimrc  
  echo "let g:ctrlsf_default_root = 'cwd'" >> ~/.vimrc  
  echo "nmap <C-F>f <Plug>CtrlSFPrompt " >> ~/.vimrc  
  echo "xmap <C-F>f <Plug>CtrlSFVwordPath" >> ~/.vimrc  
  echo "xmap <C-F>F <Plug>CtrlSFVwordExec" >> ~/.vimrc  
  echo "nmap <C-F>n <Plug>CtrlSFCwordPath" >> ~/.vimrc  
  echo "nnoremap <C-F>o :CtrlSFOpen<CR>" >> ~/.vimrc  
  echo "nnoremap <C-F>t :CtrlSFToggle<CR>" >> ~/.vimrc  

  #echo "\" =============== YouCompleteMe  ================= " >> ~/.vimrc 
  #echo "set encoding=utf-8" >> ~/.vimrc  

  #Installing VIM pluggins
  vim +PluginInstall +qall

  echo -e "\033[5;36m \t ---> Installing VIM pluggins\033[m"
  #cd ~/.vim/bundle/YouCompleteMe
  #python3 install.py --all
  sudo apt install vim-youcompleteme
  vim-addon-manager install youcompleteme

else
  echo -e "\033[1;35m ---> VIM installation skipped \033[m"
fi

############################## LATEX ##############################
echo -e "\033[7;36m ######## LaTeX  ########\033[m"
echo -e "\033[1;36m Install LaTeX: Yes (1) / No (0) \033[m"
read latex_flag
if [[ $latex_flag -eq 1 ]]
then
    sudo apt-get install texlive-full  
    cd ~/.vim
    echo -e "\033[1;36m ---> Downloading vim-latex \033[m"
    wget https://sourceforge.net/projects/vim-latex/files/releases/vim-latex-1.10.0.tar.gz
    tar -xf vim-latex-1.10.0.tar.gz 
    cd ./vim-latex-1.10.0/
    cp -r ./* ../
    cd ../
    rm -r vim-latex-1.10.0
    rm vim-latex-1.10.0.tar.gz

    echo -e "\033[1;35m ---> Configuring vim-latex \033[m"
    echo "\" =============== LaTeX Suite  ================= " >> ~/.vimrc 
    echo "filetype plugin on" >> ~/.vimrc 
    echo "filetype indent on" >> ~/.vimrc 
    echo "let g:tex_flavor='latex'" >> ~/.vimrc 
    echo "let g:Tex_GotoError=0" >> ~/.vimrc 

    echo "set sw=2" >> ~/.vim/ftplugin/tex.vim
    echo "set iskeyword+=:" >> ~/.vim/ftplugin/tex.vim

    echo -e "\033[1;35m ---> Please setup default parameters at: \033[m"
    echo -e "\033[1;35m \t ---> ~/.vim/ftplugin/latex-suite/texrc \033[m"
    echo -e "\033[1;36m \t \t Add line 98: \033[1;36m TexLet g:Tex_FormatDependency_pdf = 'dvi,ps,pdf'  \033[m"
    echo -e "\033[1;36m \t \t Chande line 97 to pdf: \033[1;36m TexLet g:Tex_MultipleCompileFormats = 'pdf' \033[m"

else
  echo -e "\033[1;35m ---> LaTeX installation skipped \033[m"
fi



############################## OpenCV ##############################
echo -e "\033[7;36m ######## OpenCV (vilsam 3.3.1)  ########\033[m"
echo -e "\033[1;36m Install opencv: Yes (1) / No (0) \033[m"
read opencv_flag
if [[ $opencv_flag -eq 1 ]]
then
  #Update system
  echo -e "\033[5;36m ---> Updating System \033[m"
  sudo apt update
  
  #Install dependencies
  echo -e "\033[5;36m ---> Install dependencies \033[m"
  sudo apt install build-essential cmake git pkg-config libgtk-3-dev \
      libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
      libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev \
      gfortran openexr libatlas-base-dev python3-dev python3-numpy \
      libtbb2 libtbb-dev libdc1394-22-dev
  #Clone GitHub Repository
  cd ~/Code
  file=./Opencv_3.3.1
  if [[ ! -d "$file" ]]
  then
    echo -e "\033[5;36m ---> Creating: $file \033[m"
    mkdir ./Opencv_3.3.1
  fi
  cd Opencv_3.3.1

  file=./opencv
  if [[ ! -d "$file" ]]
  then
    echo -e "\033[5;36m ---> Cloning: $file \033[m"
    #git clone https://github.com/opencv/opencv.git
    git clone https://github.com/opencv/opencv.git
    cd opencv
    git checkout 3.3.1 -b v3.3.1
    cd ..
  fi
  file=./opencv_contrib
  if [[ ! -d "$file" ]]
  then
    echo -e "\033[5;36m ---> Cloning: $file \033[m"
    #git clone https://github.com/opencv/opencv_contrib.git
    git clone https://github.com/opencv/opencv_contrib.git
    cd opencv_contrib
    git checkout 3.3.1 -b v3.3.1
    cd ..
  fi

  #cd ./opencv
  echo -e "\033[5;36m ---> Preparing to build \033[m"
  file=./opencv_build
  if [[ ! -d "$file" ]]
  then
    echo -e "\033[5;36m ---> Creating: $file \033[m"
    mkdir opencv_build
    cd opencv_build
    mkdir ../opencv_install

  #else 
    #mkdir build 
    #echo -e "\033[5;36m ---> Cleanning: $file \033[m"
    #cd build
    #rm -r *
  fi
  #cd build
  echo -e "\033[5;36m ---> Building Opencv \033[m"
  echo -e "\033[5;36m \t ---> Flags: \033[m"
  #General Flags
  #cmake -D CMAKE_BUILD_TYPE=RELEASE \
  #    -D CMAKE_INSTALL_PREFIX=/usr/local \
  #    -D INSTALL_C_EXAMPLES=ON \
  #    -D INSTALL_PYTHON_EXAMPLES=ON \
  #    -D OPENCV_GENERATE_PKGCONFIG=ON \
  #    -D OPENCV_EXTRA_MODULES_PATH=~/Code/Opencv_github/opencv_contrib/modules \
  #    -D BUILD_EXAMPLES=ON ..

  #VILSAM Flags
  #cmake -D CMAKE_BUILD_TYPE=RELEASE \
  #    -D CMAKE_INSTALL_PREFIX=../opencv_install \
  #    -D INSTALL_C_EXAMPLES=ON \
  #    -D INSTALL_PYTHON_EXAMPLES=ON \
  #    -D WITH_TBB=ON \
  #    -D WITH_V4L=ON \
  #    -D OPENCV_PYTHON3_INSTALL_PATH=$cwd/OpenCV-py3/lib/python3.5/site-packages \
  #    -D WITH_QT=ON \
  #    -D WITH_OPENGL=ON \
  #    -D OPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules \
  #    -D BUILD_EXAMPLES=ON ../opencv
  
  #Custom Flags
  cmake -D WITH_TBB=ON \
        -D WITH_V4L=ON \
        -D WITH_OPENMP=ON \
        -D WITH_IPP=ON \
        -D CMAKE_BUILD_TYPE=RELEASE \
        -D BUILD_EXAMPLES=OFF \
        -D WITH_NVCUVID=ON \
        -D WITH_CUDA=ON \
        -D BUILD_DOCS=OFF \
        -D BUILD_PERF_TESTS=OFF \
        -D BUILD_TESTS=OFF \
        -D WITH_CSTRIPES=ON \
        -D WITH_OPENCL=ON \
        -D WITH_QT=ON \
        -D WITH_OPENGL=ON \
        -D OPENCV_PYTHON3_INSTALL_PATH=$cwd/OpenCV-py3/lib/python3.5/site-packages \
        -D OPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules \
        -D CMAKE_INSTALL_PREFIX=../opencv_install ../opencv

  make -j8
  sudo make -j8 install
else
    echo -e "\033[1;35m ---> OpenCV installation skipped \033[m"
fi

############################## ROS ##############################
echo -e "\033[7;36m ######## ROS ########\033[m"
echo -e "\033[1;36m Install ROS: Yes (1) / No (0) \033[m"
read ros_flag
if [[ $ros_flag -eq 1 ]]
then
    echo -e "\033[5;36m ROS version: \033[m"
    read ros_version
    echo -e "\033[5;36m ---> Setting up ROS sources.list \033[m"
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

    echo -e "\033[5;36m ---> Setting up ROS keys \033[m"
    sudo apt install curl # if you haven't already installed curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

    echo -e "\033[5;36m ---> Updating System \033[m"
    sudo apt update

    #installing ros desktop be aware of the versio
    echo -e "\033[5;36m ---> Installing ROS: \033[7;36m $ros_version \033[0m"
    eval "sudo apt install ros-$ros_version-desktop-full"

    #check install
    echo -e "\033[5;36m ---> Check ROS installation \033[m"
    eval "apt search ros-$ros_version"

    #set up environment
    echo -e "\033[5;36m ---> Setting up environment \033[m"
    echo "source /opt/ros/$ros_version/setup.bash" >> ~/.bashrc
    source ~/.bashrc

    #Install extra packages
    eval "sudo apt-get install -y ros-$ros_version-navigation"
    eval "sudo apt-get install -y ros-$ros_version-robot-localization"
    eval "sudo apt-get install -y ros-$ros_version-robot-state-publisher"
    eval "sudo apt-get install -y ros-$ros_version-velodyne"

    echo -e "\033[1;33m ---> Configure catkin_ws (default) \033[m"
    cd ~
    mkdir catkin_ws
    cd catkin_ws
    mkdir src
    catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3

    echo "source devel/setup.bash" >> ~/.bashrc

else
    echo -e "\033[1;35m ---> ROS installation skipped \033[m"
fi

