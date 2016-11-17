# log4cxx library symbols missing
```
brew edit log4cxx
```

edit the configure step to look like this (add the last two lines):

```
system "./configure", "--disable-debug", "--disable-dependency-tracking",
                          "--prefix=#{prefix}",
                          # Docs won't install on macOS
                          "--disable-doxygen",
                          "--with-apr=/usr/local/Cellar/apr/1.5.2_3/bin/",
                          "--with-apr-util=/usr/local/Cellar/apr-util/1.5.4_4/bin/"
```

brew reinstall --build-from-source log4cxx
