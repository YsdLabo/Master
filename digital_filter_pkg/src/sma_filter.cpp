#include "sma_filter.h"

// コンストラクタ
SMA::SMA(int n)
{
	init(n);
}

// デストラクタ
SMA::~SMA()
{
	delete buf;    // 動的確保したメモリは解放する
}

// 初期化メソッド
void SMA::init(int n)
{
}

// 実行メソッド
void SMA::run()
{
	// /imu_rawを購読
	// /imu_smaに配信
}

// コールバック関数
void SMA::imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
	imu_raw = *msg;    // メッセージの受け取り
	
	// バッファ内データの更新
	
	
	// 移動平均の計算
	
	
	publication();
}

// メッセージの配信
void SMA::publication()
{
}
