package ahrs

import (
	"math"
	"math/rand"
	"testing"
)

func TestMathSingle32(t *testing.T) {
	rng := rand.New(rand.NewSource(1))
	testAbs32(t, 100, rng)
}

func testAbs32(t testing.TB, n int, rng *rand.Rand) {
	const tol = 1e-6

	for i := 0; i < n; i++ {
		x := rng.ExpFloat64()
		got := float64(abs_32(float32(x)))
		expect := math.Abs(x)
		if math.Abs(expect-got) > tol {
			t.Errorf("expected %g from %g, got %g", expect, x, got)
		}
	}
}

func TestAtan232(t *testing.T) {
	const (
		tol = 1e-3
		N   = 100
	)
	rng := rand.New(rand.NewSource(1))
	for i := 0; i < N; i++ {
		x, y := rng.Float64(), rng.Float64()
		got := float64(atan2_32(float32(y), float32(x)))
		expect := math.Atan2(y, x)
		if math.Abs(expect-got) > tol {
			t.Errorf("expected %g from %g, got %g", expect, x, got)
		}
	}
}

func TestSin32(t *testing.T) {
	const (
		tol = 1e-2
		N   = 100
	)
	rng := rand.New(rand.NewSource(1))
	for i := 0; i < N; i++ {
		x := rng.Float64()
		got := float64(sin_32(float32(x)))
		gotneg := float64(sin_32(-float32(x)))
		expect := math.Sin(x)
		expectneg := math.Sin(-x)
		if math.Abs(expect-got) > tol {
			t.Errorf("expected %g from %g, got %g", expect, x, got)
		}
		if math.Abs(expectneg-gotneg) > tol {
			t.Errorf("expected %g from %g, got %g", expectneg, -x, gotneg)
		}
	}
}

func TestCos32(t *testing.T) {
	const (
		tol = 1e-2
		N   = 100
	)
	rng := rand.New(rand.NewSource(1))
	for i := 0; i < N; i++ {
		x := rng.Float64()
		got := float64(cos_32(float32(x)))
		gotneg := float64(cos_32(-float32(x)))
		expect := math.Cos(x)
		expectneg := math.Cos(-x)
		if math.Abs(expect-got) > tol {
			t.Errorf("expected %g from %g, got %g", expect, x, got)
		}
		if math.Abs(expectneg-gotneg) > tol {
			t.Errorf("expected %g from %g, got %g", expectneg, -x, gotneg)
		}
	}
}

func randRange(r *rand.Rand, x1, x2 float64) float64 {
	return x1 + (r.Float64()-.5)*(x2-x1)
}
