%%
% AIAA class to format CU Aerospace Engineering Papers and Lab Reports in the AIAA standard
%
% Written by Dylan Stewart
% Dylan.Stewart@Colorado.edu
%
%Revision Tracking
%r1.0 - Initially written
%r1.1 - Added more figure functionality, table function, and fixed an issue with headers acting strangely and running into text
%r1.2 - Removed title modifications, the built in ones are sufficient and less problematic
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%Function Listing
%
% \matlabcode{filename.m}     		 Function to include matlab source code with appropriate syntax coloring
%
% Equation stuff:
%	\neweq{equation handle}{equation} 		Function to add a new equation with a reference handle
%	\neweq*{equation}					Function to add a new equation without numbering for reference
%	\eq{equation handle}					Inserts Eq. # with auto equation numbering
%	\eqs{equation handle}{equation handle}	Inserts Eqs. # & # with auto equation numbering
%	\Eq{equation handle}					Inserts Equation # with auto equation numbering for the start of sentences
%
%	Tables:
%
%	\newtable{table handle}{column formatting}{caption}{table formatting}	Inserts a new table with column formatting specified as c,r,l for each row specifying the text alignment and vertical bars | for vertical lines in table. The proper table formatting is as follows
%	\newtable{traptable}{|c|c|c|}{Trapezium Method of Approximation}{
%		\hline
%		Number of Samples & Approximated Value & Error\\
%		\hline
%		20 & 7.486 & -3.766\\
%		\hline
%		40 & 3.862 & -1.459\\
%		\hline
%		100 & 1.569 & 0.001\\
%		\hline
%		}
%	
%	\tableref{table handle}  				Inserts Table # with auto table numbering
%
%	Figures:
%
%	\rightfig{file path}{figure handle}{caption}	Inserts a text wrapped figure on the right side of the page
%	\leftfig{filepath}{figure handle}{caption}	Inserts a text wrapped figure on the left side of the page
%	\centerfig{filepath}{figure handle}{caption}	Inserts a figure in the center of the page
%	\sidebyside{filepath}{figure handle}{caption}{filepath}{figure handle}{caption}	Inserts two figures side by side on the page
%	\fig{figure handle}					Inserts Fig. # with auto figure numbering
%	\Figure{figure handle}				Inserts Figure # with auto figure numbering for the beginning of sentences
%	\figs{figure handle}{figure handle}		Inserts Figs. # & # with auto figure numbering
%




\NeedsTeXFormat{LaTeX2e}[1994/12/01]

\ProvidesClass{aiaa}[2014/9/22 v1.2 CU Aerospace Engineering Lab Report]

\DeclareOption*{\PassOptionsToClass{\CurrentOption}{article}%
                \typeout{Unknown option: passing ``\CurrentOption"
                         option on to the underlying LaTeX article class.}}
\ProcessOptions

\LoadClass[letterpaper,twoside,10pt]{article}

%%
% Font

\RequirePackage{times}

%%
% Page layout

\RequirePackage[tmargin=0.25in,
                headheight=0.50in,
                headsep=0.25in,
                lmargin=1in,
                rmargin=1in,
                footskip=0.25in,
                bmargin=0.75in]{geometry}

%%
% Set matlab code environment

\RequirePackage[numbered,autolinebreaks,useliterate]{mcode}
\lstset{breakatwhitespace=false}

\newcommand{\matlabcode}[1]{
	\lstinputlisting{#1}
}

%%
% Footnotes

\renewcommand\thefootnote{\@fnsymbol\c@footnote}

%%
% Page style (headers and footers)

\RequirePackage{fancyhdr}% simple control of headers and footers
\RequirePackage{lastpage}
\fancyhf{}
\renewcommand{\headrulewidth}{0pt}
\cfoot{\footnotesize\normalfont
       \thepage~of~\pageref{LastPage}\\
       \rule[.2\baselineskip]{0.5in}{0.2pt}\\
        University of Colorado at Boulder}
\pagestyle{fancy}

%%
% Abstract environment

\renewenvironment{abstract}%
  {\small\quotation\bfseries}%
  {\normalfont\endquotation\normalsize}


%%
% Equation Formatting
\RequirePackage{suffix}
\newcommand{\neweq}[2]{
\begin{equation}
 \label{e:#1}
 #2
\end{equation}
}

\WithSuffix\newcommand\neweq*[1]{
	$$
		#1
	$$
}

\newcommand{\eq}[1]{Eq. \ref{e:#1}}

\newcommand{\eqs}[2]{Eqs. \ref{e:#1} \& \ref{e:#2}}

\newcommand{\Eq}[1]{Equation \ref{e:#1}}

%%
% Tables
\RequirePackage{float}
\restylefloat{table}
\newcommand{\newtable}[4]{

\begin{table}
	\begin{center}
		\caption{#3}
		\begin{tabular}{#2}
			#4
		\end{tabular}
		\label{t:#1}
	\end{center}
\end{table}

}

\newcommand{\tableref}[1]{
Table \ref{t:#1}}

% Pretty Fractions

\RequirePackage{amsfonts}
\RequirePackage{amsmath}

%%
% Images

\RequirePackage{graphicx}
\RequirePackage{wrapfig}
\RequirePackage{float}

\setkeys{Gin}{ width=\linewidth, totalheight=\textheight, keepaspectratio }

% Text wrapped figures
\newcommand{\rightfig}[3]{
\begin{wrapfigure}{R}{.45\textwidth}
	\includegraphics[width=.4\textwidth]{#1}
	\caption{#3}
	\label{f:#2}
\end{wrapfigure}
}

\newcommand{\leftfig}[3]{
\begin{wrapfigure}{L}{.45\textwidth}
	\includegraphics[width=.4\textwidth]{#1}
	\caption{#3}
	\label{f:#2}
\end{wrapfigure}
}

\newcommand{\sidebyside}[6]{
	\begin{figure}[H]
		\begin{minipage}[b]{0.49\linewidth}
			\centering
			\includegraphics[width=\textwidth]{#1}
			\caption{#3}
			\label{f:#2}
		\end{minipage}
		\hspace{0.05\linewidth}
		\begin{minipage}[b]{0.49\linewidth}
			\centering
			\includegraphics[width=\textwidth]{#4}
			\caption{#6}
			\label{f:#5}
		\end{minipage}
	\end{figure}
}

\newcommand{\centerfig}[3]{
	\begin{figure}[H]
		{\centering
			\includegraphics[width=.85\textwidth]{#1}
			\caption{#3}
			\label{f:#2}
		}
	\end{figure}
}

\newcommand{\fig}[1]{Fig. \ref{f:#1}}

\newcommand{\Figure}[1]{Figure \ref{f:#1}}

\newcommand{\figs}[2]{Figs. \ref{f:#1} \& \ref{f:#2}}

%%
% Captions - original from latex.ltx
\long\def\@makecaption#1#2{%
  \vskip\abovecaptionskip
  \sbox\@tempboxa{\footnotesize\bfseries #1\@. #2}%
  \ifdim \wd\@tempboxa >\hsize
    \footnotesize\bfseries #1\@. #2\par
  \else
    \global \@minipagefalse
    \hb@xt@\hsize{\hfil\box\@tempboxa\hfil}%
  \fi
  \vskip\belowcaptionskip}
\setlength\belowcaptionskip{5pt plus 1pt minus 1pt}

%%
% Section headers - original from article.cls

\setcounter{secnumdepth}{3}

\renewcommand\thesection      {\@Roman\c@section.}
\renewcommand\thesubsection   {\@Alph\c@subsection.}
\renewcommand\thesubsubsection{\@arabic\c@subsubsection.}

\renewcommand\section{\@startsection {section}{1}{\z@}%
                                     {3ex}%
                                     {2ex}%
                                     {\center\normalfont\large\bfseries}}
\renewcommand\subsection{\@startsection{subsection}{2}{\z@}%
                                       {3ex}%
                                       {2ex}%
                                       {\normalfont\normalsize\bfseries}}
\renewcommand\subsubsection{\@startsection{subsubsection}{3}{\z@}%
                                       {3ex}%
                                       {2ex}%
                                       {\normalfont\normalsize\itshape}}
\renewcommand\paragraph{\@latex@error{Section level \noexpand\paragraph unavailable}\@eha}
\renewcommand\subparagraph{\@latex@error{Section level \noexpand\subparagraph unavailable}\@eha}

%%
% Colors

\RequirePackage{color}
  \definecolor{darkblue}{rgb}{0.0,0.0,0.5}

%%
% Hyperlinks

\RequirePackage[hyperfootnotes=false,
                hyperfigures,
                breaklinks,
                colorlinks,
                urlcolor=darkblue,
                filecolor=darkblue,
                citecolor=black,
                linkcolor=black,
                pagecolor=black,
                menucolor=black,
                anchorcolor=black]{hyperref}

%%
% Floats -- make 'h' placement default

\def\fps@figure{htbp}
\def\fps@table{htbp}


%%
% References

\RequirePackage[numbers,sort&compress]{natbib}
%\RequirePackage{hypernat}
  \bibpunct{}{}{,}{s}{}{} % superscript citation

% superscript reference list -- original from article.cls
\renewcommand{\@biblabel}[1]{\textsuperscript#1}
\renewenvironment{thebibliography}[1]%
     {\section*{\refname}
      \list{\@biblabel{\@arabic\c@enumiv}}%
           {\setlength{\leftmargin}{0pt}%
            \settowidth{\labelwidth}{\@biblabel{#1}}%
            \setlength{\itemindent}{\parindent}%
            \advance\itemindent by \labelwidth%
            \setlength{\labelsep}{0.0em}%
            \setlength{\itemsep}{-\smallskipamount}%
            \@openbib@code%
            \usecounter{enumiv}%
            \let\p@enumiv\@empty%
            \renewcommand\theenumiv{\@arabic\c@enumiv}%
            \footnotesize}%
      \sloppy
      \clubpenalty4000
      \@clubpenalty \clubpenalty
      \widowpenalty4000%
      \sfcode`\.\@m}
      {\def\@noitemerr
        {\@latex@warning{Empty `thebibliography' environment}}%
     \endlist}
