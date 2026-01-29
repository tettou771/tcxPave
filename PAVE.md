# Pave 設計思想

Paveは baku89 氏が開発した、SVG/Path2D曲線を操作するための環境非依存ツールキット。
関数型プログラミングの思想に基づいて設計されており、ステートレスかつ合成可能なAPIを提供する。

GitHub: https://github.com/baku89/pave

---

## コアコンセプト

### 1. イミュータブルなデータ構造

パスデータはプレーンなオブジェクトとして表現され、イミュータブル（不変）に扱われる。

```typescript
// パスはシンプルなオブジェクト
type Path = {
  curves: Curve[]
}

type Curve = {
  vertices: Vertex[]
  closed: boolean
}

type Vertex = {
  point: vec2
  command: 'L' | 'C' | 'A'  // Line, CubicBezier, Arc
  args?: CommandArgs
}
```

すべての操作は元のデータを変更せず、新しいデータを返す。

### 2. 関数型API

プロパティアクセスではなく、関数呼び出しでパスの情報を取得する。

```typescript
// ❌ OOPスタイル（Paper.jsなど）
path.length
path.bounds
path.getNormalAt(0.5)

// ✅ Paveスタイル
Path.length(path)
Path.bounds(path)
Path.normalAtTime(path, 0.5)
```

この設計により：
- 関数の合成が容易
- Tree-shakingが効きやすい
- テストが書きやすい

### 3. 環境非依存

Canvas API、SVG、p5.js、Paper.js など任意のJavaScript環境で動作する。
出力形式を変換する関数を提供：

```typescript
const circle = Path.circle([0, 0], 100)

// SVG用
const d = Path.toSVG(circle)
svgPathElement.setAttribute('d', d)

// Canvas API用
const path2d = Path.toPath2D(circle)
context.stroke(path2d)
```

---

## Distortシステム

Paveの最も強力な機能の一つ。パスに対して変形（distortion）を適用できる。

### Distort型の定義

```typescript
type Distort = (p: vec2) => mat2d
```

**重要**: Distortは「点を受け取って変換行列を返す関数」として定義されている。
単なる点→点の変換ではなく、各点における局所的な変換行列を返すことで、
接線やノーマルの情報も正しく変換できる。

### カスタムDistortの作成

`fromPointTransformer`を使うと、シンプルな点変換関数からDistortを生成できる：

```typescript
// 点変換関数: (p: vec2) => vec2
// Distort関数: (p: vec2) => mat2d

export function fromPointTransformer(fn: (p: vec2) => vec2): Distort {
  const delta = 0.01

  return (p: vec2) => {
    const pt = fn(p)
    const deltaX = fn(vec2.add(p, [delta, 0]))
    const deltaY = fn(vec2.add(p, [0, delta]))

    // 数値微分でヤコビアンを計算
    const axisX = vec2.scale(vec2.sub(deltaX, pt), 1 / delta)
    const axisY = vec2.scale(vec2.sub(deltaY, pt), 1 / delta)

    return [...axisX, ...axisY, ...pt]  // mat2d形式
  }
}
```

この設計のポイント：
- ユーザーは単純な `(p: vec2) => vec2` 関数を書くだけでいい
- 内部で数値微分によりヤコビアン（局所的な変換行列）を自動計算
- ベジェ曲線の制御点も正しく変換される

### 使用例

```typescript
// 組み込みのwave distort
const waved = Path.distort(circle, Distort.wave(5, 20))

// 完全カスタムのdistort
const myDistort = Distort.fromPointTransformer(p => {
  const [x, y] = p
  return [x + Math.sin(y * 0.1) * 10, y]
})
const custom = Path.distort(circle, myDistort)

// noise使った例
const noiseDistort = Distort.fromPointTransformer(p => {
  const [x, y] = p
  const n = noise2D(x * 0.05, y * 0.05)
  return [x + n * 15, y + n * 15]
})
```

### 組み込みDistort関数

#### wave
```typescript
function wave(
  amplitude: number,    // 波の振幅
  width: number,        // 波長
  phase: number = 0,    // 位相（度）
  angle: number = 0,    // 波の方向（度）
  origin: vec2 = [0, 0] // 原点
): Distort
```

#### twirl
```typescript
function twirl(
  center: vec2,                        // 中心点
  radius: number,                      // 半径
  angle: number,                       // 回転角度（度）
  ramp: (t: number) => number = t => t // カスタムイージング関数
): Distort
```

`ramp`パラメータにより、中心からの距離に応じた回転量のカーブをカスタマイズできる。

---

## パス操作の主要関数

### プリミティブ生成
```typescript
Path.circle(center: vec2, radius: number): Path
Path.rectangle(topLeft: vec2, size: vec2): Path
Path.regularPolygon(center: vec2, radius: number, sides: number): Path
Path.line(start: vec2, end: vec2): Path
Path.arc(center: vec2, radius: number, startAngle: number, endAngle: number): Path
```

### 計測・取得
```typescript
Path.length(path: Path): number
Path.bounds(path: Path): Rect
Path.pointAtTime(path: Path, t: number): vec2
Path.tangentAtTime(path: Path, t: number): vec2
Path.normalAtTime(path: Path, t: number): vec2
```

### 変換
```typescript
Path.transform(path: Path, matrix: mat2d): Path
Path.offset(path: Path, distance: number, options?: OffsetOptions): Path
Path.distort(path: Path, distort: Distort): Path
Path.resample(path: Path, interval: number): Path
```

### 入出力
```typescript
Path.fromSVG(d: string): Path
Path.toSVG(path: Path): string
Path.toPath2D(path: Path): Path2D
```

---

## 移植時の考慮事項

### 必須の依存概念

1. **vec2**: 2D座標 `[x, y]`
2. **mat2d**: 2D変換行列（アフィン変換）`[a, b, c, d, tx, ty]`
3. **ベジェ曲線の計算**: de Casteljauアルゴリズムなど

### 移植のアプローチ

1. **最小構成から始める**
   - vec2, mat2d の基本演算
   - Path/Curve/Vertex のデータ構造
   - 基本的なプリミティブ生成（circle, rectangle）

2. **Distortシステム**
   - `fromPointTransformer` は数値微分なので言語問わず実装可能
   - 微分のδ値（0.01）は調整が必要かも

3. **レンダリングは環境に合わせる**
   - 各言語/フレームワークの描画APIに合わせて出力関数を実装

### C++（TrussC）への移植ヒント

```cpp
// vec2
using vec2 = std::array<float, 2>;

// mat2d (row-major)
struct mat2d {
    float a, b, c, d, tx, ty;
};

// Distort型
using Distort = std::function<mat2d(vec2)>;

// fromPointTransformer
Distort fromPointTransformer(std::function<vec2(vec2)> fn) {
    constexpr float delta = 0.01f;
    
    return [fn, delta](vec2 p) -> mat2d {
        vec2 pt = fn(p);
        vec2 deltaX = fn({p[0] + delta, p[1]});
        vec2 deltaY = fn({p[0], p[1] + delta});
        
        float axisX_x = (deltaX[0] - pt[0]) / delta;
        float axisX_y = (deltaX[1] - pt[1]) / delta;
        float axisY_x = (deltaY[0] - pt[0]) / delta;
        float axisY_y = (deltaY[1] - pt[1]) / delta;
        
        return {axisX_x, axisX_y, axisY_x, axisY_y, pt[0], pt[1]};
    };
}
```

---

## 参考リンク

- [Pave Documentation](https://baku89.github.io/pave/)
- [Pave Sandbox](https://baku89.github.io/pave/sandbox.html) - インタラクティブに試せる
- [GitHub Repository](https://github.com/baku89/pave)
- 依存ライブラリ:
  - [Bezier.js](https://pomax.github.io/bezierjs/) by Pomax
  - [Paper.js](http://paperjs.org/) by Jürg Lehni
