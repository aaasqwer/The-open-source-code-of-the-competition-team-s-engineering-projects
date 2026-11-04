本工程直接使用 ref_clock.v 中的 eth_ref_clock 模块。它保留板卡千兆网口例程的 GPLL 参数，通过 PDS 自带 GTP_PLL_E2 原语实现；工程不依赖另外生成的网口 IP。
