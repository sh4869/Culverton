Culverton
---

MicroMouse 2017

# Dependencies

* [STM32CubeMX](http://www.st.com/ja/development-tools/stm32cubemx.html)
    * STM32の初期化用
* [GCC ARM Toolchain](https://launchpad.net/gcc-arm-embedded/5.0/5-2015-q4-major/+download/gcc-arm-none-eabi-5_2-2015q4-20151219-win32.zip)(直接のリンクです)
    * コンパイラ
    * かならずインストール先のbinフォルダにパスを通すようにしてください。パスの通し方は[こちらのサイト](http://realize.jounin.jp/path.html)などを参考にするとよいです。
* [BuildTool](https://github.com/gnuarmeclipse/windows-build-tools/releases/download/v2.6/gnuarmeclipse-build-tools-win32-2.6-201507152002-setup.exe)
    * Makeコマンドなどが入っています
    * こちらもかならずインストールしたディレクトリにパスを通すようにしてください。

# Usage 

1. Dependenciesをそれぞれ揃える
2. STM32CubeMXでCulverton.iocを開き、Generate Codeを選択してソースコードを生成する
3. cmd.exeを実行し、「cd」コマンドでプログラムがあるディレクトリまで移動し、makeコマンドを実行する

## Culverton

機体のプログラム。随時更新中。STM32CubeMXによって生成されるファイルは無視しています。Culverton.iocをSTM32CUbeMXで起動すれば同じ環境が再現されるはず……。