set nocompatible
syntax on
set hidden
set showmode
set wildmenu

" Change map leader
let mapleader = ","
let g:mapleader = ","

" Fast saving
nmap <leader>w :w!<cr>

nmap <silent> ,ev :e $MYVIMRC<cr>
nmap <silent> ,sv :so $MYVIMRC<cr>
nmap  ,p :set invpaste:set paste?
nmap  ,n :set invhls:set hls?
map  ,w :set invwrap:set wrap?
nmap  ,rr :1,$retab
nmap  ,cd :lcd %:h
nmap  ,md :!mkdir -p %:p:h
filetype off

set rtp+=~/.vim/bundle/vundle/
call vundle#rc()

Bundle 'gmarik/vundle'
Bundle 'flazz/vim-colorschemes'
Bundle 'LaTeX-Box-Team/LaTeX-Box'
Bundle 'sudar/vim-arduino-syntax'
Bundle 'jwhitley/vim-matchit'
Bundle 'MarcWeber/vim-addon-mw-utils'
Bundle 'tomtom/tlib_vim'
Bundle 'garbas/vim-snipmate'
Bundle "honza/vim-snippets"
Bundle 'tpope/vim-surround'
Bundle 'scrooloose/nerdtree'
Bundle 'jistr/vim-nerdtree-tabs'
Bundle 'kien/ctrlp.vim'
Bundle 'vim-scripts/taglist.vim'

filetype plugin indent on

" IMPORTANT: win32 users will need to have 'shellslash' set so that latex
" can be called correctly.
set shellslash

" Vim settings
" Set the search scan to wrap around the file
set wrapscan
" Command line height
set ch=1
" Visual bell
set vb
" Allow backspace over indent, eol, and start of insert
set backspace=2
" Status Line
set stl=%f\ %m\ %r\ Line:\ %l/%L[%p%%]\ Col:\ %c\ Buf:\ #%n\ [%b][0x%B]

"Tell vim to put a status line
set laststatus=2
" Hide mouse while typing
set mousehide
set history=100
:set virtualedit=all
set complete=.,w,b,t
set incsearch

" Set cursor line
set cul
hi CursorLine guibg=#2d2d2sd

" Turn backup off
set nobackup
set nowb
set noswapfile

" Use spaces instead of tabs
set expandtab
set smarttab
set shiftwidth=4
set ai
set si
set wrap

map j gj
map k gk

set t_Co=256
set background=dark

" Taglist Settings
noremap <silent> <Leader>t :TlistToggle<CR>
"
" Taglist plugin config
let Tlist_Use_Right_Window = 1
let Tlist_Inc_Winwidth = 0
let Tlist_WinWidth = 45
let Tlist_GainFocus_On_ToggleOpen= 1
let Tlist_Ctags_Cmd = 'ctags'
let Tlist_Show_One_File = 1

" NERDTree settings
:map <F5> :NERDTreeToggle<CR>

" Mouse Mode iterm2
set mouse=a
set ttym=xterm2

" ctrlp hotkeys
let g:ctrlp_map = '<c-p>'
let g:ctrlp_cmd = 'CtrlP'
let g:ctrlp_working_path_mode = 'ra'
set wildignore+=*/tmp/*,*.so,*.swp,*.zip     " MacOSX/Linux

let g:ctrlp_custom_ignore = '\v[\/]\.(git|hg|svn)$'
let g:ctrlp_user_command = 'find %s -type f'        " MacOSX/Linux
