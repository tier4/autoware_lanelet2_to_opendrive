# 使い方ガイド

このガイドでは、`autoware-lanelet2-to-opendrive`パッケージを使用してLanelet2マップをOpenDRIVE形式に変換する方法を説明します。

## 基本的な使い方

### CLIコマンド

パッケージには2つのCLIコマンドが含まれています:

#### 1. `convert` - Lanelet2からOpenDRIVEへの変換

```bash
convert input.osm --preprocess-config config.yaml
```

#### 2. `preprocess-lanelet` - Lanelet2マップの前処理

```bash
preprocess-lanelet config.yaml
```

## 変換コマンドの詳細

### 基本的な変換例

最もシンプルな使用例:

```bash
convert input_map.osm --preprocess-config config.yaml
```

このコマンドは`input_map.xodr`を生成します(デフォルトの出力ファイル名)。

### 出力ファイル名を指定する

```bash
convert input_map.osm --preprocess-config config.yaml -o output_map.xodr
```

または

```bash
convert input_map.osm --preprocess-config config.yaml --output output_map.xodr
```

### 詳細ログを有効にする

```bash
convert input_map.osm --preprocess-config config.yaml -v
```

または

```bash
convert input_map.osm --preprocess-config config.yaml --verbose
```

## コマンドラインオプション

### `convert`コマンドのオプション

| オプション | 短縮形 | 説明 | 必須 |
|-----------|--------|------|------|
| `lanelet2_file` | - | 入力Lanelet2 OSMファイルのパス | ✓ |
| `--preprocess-config` | - | 前処理設定YAMLファイルのパス(MGRS コードを含む) | ✓ |
| `--output` | `-o` | 出力OpenDRIVEファイルのパス(デフォルト: input_file.xodr) | |
| `--verbose` | `-v` | 詳細なログ出力を有効にする | |

### `preprocess-lanelet`コマンドのオプション

| オプション | 短縮形 | 説明 | 必須 |
|-----------|--------|------|------|
| `config` | - | YAML設定ファイルのパス | ✓ |
| `--mgrs` | - | MGRS コード(設定ファイルを上書き) | |
| `--dry-run` | - | 出力を保存せずに実行(検証のみ) | |
| `--verbose` | `-v` | 詳細なログ出力を有効にする | |
| `--output-config` | - | 読み込んだ設定を新しいYAMLファイルに保存 | |

## 入力ファイルの要件

### Lanelet2マップの要件

入力として有効なLanelet2 OSMファイルが必要です:

- **ファイル形式**: `.osm` 形式のLanelet2マップ
- **座標系**: MGRS座標系で定義されたマップ
- **必須要素**:
  - レーンレット(lanelet)要素
  - ラインストリング(linestring)要素
  - ポイント(point)要素
- **属性**: Autoware用の標準Lanelet2属性

### 前処理設定ファイル(YAML)

変換には前処理設定ファイルが必須です。このファイルにはMGRSコードと、オプションで前処理操作を含めることができます。

#### 最小限の設定例

```yaml
input_map_path: /path/to/input.osm
output_map_path: /path/to/preprocessed.osm
mgrs_code: 54SUE815501

# 前処理操作なし(変換のみMGRSコードを使用)
```

#### 前処理操作を含む設定例

```yaml
input_map_path: /path/to/input.osm
output_map_path: /path/to/preprocessed.osm
mgrs_code: 54SUE815501

# レーンレットのマージ
merge_operations:
  - lanelet_ids: [100, 101, 102]
    validate: true
    tolerance: 0.001

# レーンレットの削除
remove_lanelet_operations:
  - lanelet_ids: [300, 301]

# turn_direction属性の削除(全レーンレットから)
remove_turn_direction_operations:
  - lanelet_ids: []  # 空リスト = すべてのレーンレットから削除

# グローバル設定
dry_run: false
verbose: true
```

#### 利用可能な前処理操作

1. **merge_operations**: 複数のレーンレットを1つにマージ
2. **remove_operations**: 古い形式のレーンレット削除
3. **replace_operations**: レーンレットを置換
4. **validate_operations**: レーンレットの連続性を検証
5. **move_point_operations**: ポイントの座標を移動
6. **delete_point_operations**: ポイントを削除
7. **remove_lanelet_operations**: レーンレット全体を削除
8. **remove_turn_direction_operations**: turn_direction属性を削除

## 出力ファイル

### OpenDRIVE形式

変換後、以下の特徴を持つOpenDRIVE形式(.xodr)ファイルが生成されます:

- **OpenDRIVEバージョン**: 1.4
- **座標系**: MGRS座標系(入力マップと同じ)
- **含まれる要素**:
  - Roads(道路): 通常の道路とジャンクション接続道路
  - Junctions(交差点): 交差点領域と接続情報
  - Signals(信号): Lanelet2マップから抽出された交通信号
  - Controllers(コントローラー): 信号機コントローラー
- **ターゲット**: CARLAシミュレーター用に最適化

### 出力の構造

```
output.xodr
├── header (ヘッダー情報とgeoReference)
├── roads
│   ├── 通常の道路(ジャンクション外)
│   └── 接続道路(ジャンクション内)
├── junctions (交差点とその接続)
└── controllers (信号機コントローラー)
```

## よくある使用例

### ユースケース1: シンプルな変換

前処理なしでLanelet2マップをOpenDRIVEに変換:

```bash
# 1. 最小限の設定ファイルを作成(config.yaml)
# input_map_path, output_map_path, mgrs_codeを含む

# 2. 変換を実行
convert my_map.osm --preprocess-config config.yaml -o my_map.xodr
```

### ユースケース2: 前処理を含む変換

マップの問題を修正してから変換:

```bash
# 1. 前処理操作を含む設定ファイルを作成(preprocess_config.yaml)
# merge_operations, remove_lanelet_operationsなどを含む

# 2. 前処理と変換を一度に実行
convert original_map.osm --preprocess-config preprocess_config.yaml -o fixed_map.xodr
```

### ユースケース3: 前処理のみ実行

OpenDRIVE変換の前にマップを前処理:

```bash
# 1. 前処理のみを実行
preprocess-lanelet preprocess_config.yaml

# 2. 前処理されたマップを検証
preprocess-lanelet preprocess_config.yaml --dry-run -v

# 3. 前処理されたマップを変換
convert preprocessed_map.osm --preprocess-config simple_config.yaml
```

### ユースケース4: Autoware + CARLAシミュレーション

AutowareマップをCARLAシミュレーターで使用:

```bash
# 1. AutowareのLanelet2マップを変換
convert autoware_map.osm --preprocess-config config.yaml -o carla_map.xodr

# 2. 生成されたcarla_map.xodrをCARLAにインポート
# (CARLAのドキュメントを参照)
```

### ユースケース5: デバッグと検証

詳細ログを使用して変換プロセスをデバッグ:

```bash
# 詳細ログで実行
convert input.osm --preprocess-config config.yaml -v -o output.xodr

# これにより以下の情報が表示されます:
# - 読み込まれたレーンレット、ラインストリング、ポイントの数
# - 通常の道路とジャンクション道路の構築
# - ジャンクション接続の構築
# - 信号とコントローラーの抽出
# - Road-Laneletマッピング情報
```

## Autoware統合

このパッケージはAutoware自律運転ソフトウェアで使用するために設計されています。

### 一般的なワークフロー

1. **AutowareマップのエクスポートLanelet2形式でAutowareマップを準備
2. **MGRS コードの取得**: マップの座標系に対応するMGRS コードを確認
3. **前処理設定の作成**: 必要に応じてマップの修正を定義
4. **OpenDRIVEへの変換**: `convert`コマンドを使用
5. **シミュレーターへのインポート**: 生成されたOpenDRIVEファイルをCARLAなどで使用

## シミュレーションとテスト

!!! info "現在のサポート"
    現在、このパッケージは**CARLAシミュレーター**用のOpenDRIVE形式を生成します。他のシミュレーションプラットフォームのサポートは将来のリリースで追加される可能性があります。

### CARLAシミュレーターでの使用

生成されたOpenDRIVEマップは以下に対応しています:

- **CARLAシミュレーター**(主要ターゲット)
- カスタムマップのインポート
- シミュレーション環境での自律走行テスト

## ベストプラクティス

### 入力の検証

変換前にLanelet2マップが正しい形式であることを確認してください:

```bash
# 前処理のdry-runモードで検証
preprocess-lanelet config.yaml --dry-run -v
```

### 出力の確認

生成されたOpenDRIVEファイルをターゲットアプリケーションで確認してください:

- CARLAシミュレーターでマップを読み込む
- OpenDRIVEビューアーで視覚的に確認
- XMLの構造を検証

### 問題の報告

変換の問題が発生した場合は、[GitHub Issues](https://github.com/tier4/autoware_lanelet2_to_opendrive/issues)で報告してください。

報告時には以下を含めてください:
- 入力Lanelet2マップのサンプル(可能な場合)
- 使用した前処理設定ファイル
- エラーメッセージまたは予期しない動作
- 詳細ログ出力(`--verbose`フラグを使用)

## Pythonライブラリとして使用

### パッケージのインポート

```python
import autoware_lanelet2_to_opendrive
from autoware_lanelet2_to_opendrive.main import (
    load_lanelet2_map,
    convert_lanelet2_to_opendrive,
    preprocess_and_convert
)
```

### プログラムからの変換

```python
from pathlib import Path
from autoware_lanelet2_to_opendrive.main import preprocess_and_convert

# 変換を実行
preprocess_and_convert(
    lanelet2_file=Path("input_map.osm"),
    output_file=Path("output_map.xodr"),
    preprocess_config_path=Path("config.yaml"),
    verbose=True
)
```

### 高度な使用例

```python
from pathlib import Path
from autoware_lanelet2_to_opendrive.main import (
    load_lanelet2_map,
    convert_lanelet2_to_opendrive
)

# 1. マップを読み込む
lanelet_map = load_lanelet2_map(
    Path("input.osm"),
    mgrs="54SUE815501"
)

# 2. OpenDRIVEに変換
opendrive, mapping = convert_lanelet2_to_opendrive(
    lanelet_map=lanelet_map,
    mgrs_code="54SUE815501",
    output_path=Path("output.xodr")
)

# 3. マッピング情報を使用
print(f"Roads: {len(mapping.road_to_lanelets)}")
print(f"Lanelets: {len(mapping.lanelet_to_road)}")

# 特定のレーンレットがどの道路に対応するかを確認
lanelet_id = 100
if lanelet_id in mapping.lanelet_to_road:
    road_id = mapping.lanelet_to_road[lanelet_id]
    print(f"Lanelet {lanelet_id} -> Road {road_id}")
```

## 実例

実際の使用例については、リポジトリの`examples/`ディレクトリを確認してください。

## 次のステップ

- 詳細なAPI仕様は[APIリファレンス](api.md)を参照
- 開発に貢献したい場合は[開発ガイド](development.md)を確認
- 信号と交通ルールの変換については[信号ドキュメント](signals.md)を参照

## トラブルシューティング

### よくあるエラー

#### "MGRS code must be provided"

**原因**: 前処理設定ファイルにMGRS コードが含まれていない

**解決方法**:
```yaml
# config.yamlに以下を追加
mgrs_code: 54SUE815501  # 実際のMGRS コードに置き換え
```

#### "Lanelet2 file not found"

**原因**: 入力ファイルのパスが正しくない

**解決方法**:
- ファイルパスを確認
- 絶対パスまたは相対パスを正しく指定

#### "Failed to load Lanelet2 map"

**原因**: マップファイルの形式が正しくない、またはMGRS コードが間違っている

**解決方法**:
- マップファイルがLanelet2 OSM形式であることを確認
- MGRS コードがマップの座標系と一致することを確認
- `--verbose`フラグで詳細を確認

### デバッグのヒント

1. **詳細ログを有効にする**: 常に`-v`フラグを使用
2. **段階的に実行**: 前処理と変換を分けて実行
3. **dry-runモードを使用**: 実際の変更前に検証
4. **小さなマップでテスト**: まず小規模なマップで試す
