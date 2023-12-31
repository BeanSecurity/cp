package cp

import "math"

type RotationJoint struct {
	*Constraint
	// phase, ratio float64
	// ratio_inv float64

	iSum float64

	bias, jAcc float64
}

// func NewRotationJoint(a, b *Body, phase, ratio float64) *Constraint {
func NewRotationJoint(a, b *Body) *Constraint {
	joint := &RotationJoint{
		// phase:     phase,
		// ratio:     ratio,
		// ratio_inv: 1.0 / ratio,
		jAcc: 0,
	}
	constraint := NewConstraint(joint, a, b)
	joint.Constraint = constraint
	return constraint
}

func (joint *RotationJoint) PreStep(dt float64) {
	a := joint.a
	b := joint.b

	// calculate moment of inertia coefficient.
	joint.iSum = 1.0 / (a.i_inv + b.i_inv)

	y, x := math.Sincos(b.a - a.a)
	diff := math.Atan2(y, x)

	// calculate bias velocity
	maxBias := joint.Constraint.maxBias
	joint.bias = Clamp(-bias_coef(joint.errorBias, dt)*diff/dt, -maxBias, maxBias)
}

func (joint *RotationJoint) ApplyCachedImpulse(dt_coef float64) {
	a := joint.a
	b := joint.b

	j := joint.jAcc * dt_coef
	a.w -= j * a.i_inv
	b.w += j * b.i_inv
}

func (joint *RotationJoint) ApplyImpulse(dt float64) {
	a := joint.a
	b := joint.b

	// compute relative rotational velocity
	wr := b.w - a.w

	jMax := joint.Constraint.maxForce * dt

	// compute normal impulse
	j := (joint.bias - wr) * joint.iSum
	jOld := joint.jAcc
	joint.jAcc = Clamp(jOld+j, -jMax, jMax)
	j = joint.jAcc - jOld

	// apply impulse
	a.w -= j * a.i_inv
	b.w += j * b.i_inv
}

func (joint *RotationJoint) GetImpulse() float64 {
	return math.Abs(joint.jAcc)
}
