#ifndef OC_TRANSFORM_H
#define OC_TRANSFORM_H

#include "TVector3.h"
#include "Quaternion.h"

template <typename Scaler>
struct TTransform
{
	TTransform(){}
	TTransform(const TVector3<Scaler>& inTranslation, const TQuaternion<Scaler>& inRotation) : translation(inTranslation), rotation(inRotation){}

    //Apply rotation
    TVector3<Scaler> transformVector(const TVector3<Scaler>& rhs) const
	{
		return rotation * rhs;
	}

    //Apply rotation and then translation
    TVector3<Scaler> transformPoint(const TVector3<Scaler>& rhs) const
    {
        return (rotation * rhs) + translation;
    }

    // concat so that (this * rhs).transformPoint(x) = this.transformPoint(rhs.transformPoint(x))
    TTransform<Scaler> operator*(const TTransform<Scaler>& rhs) const
    {
        return TTransform<Scaler>( translation + (rotation * rhs.translation), rotation * rhs.rotation);
    }

    //We can't get a pure inverse because we'd have to change the order of operations (need to undo translation and then undo rotation)
    //We can do this: X = A.inv() * B so that XA = B.
    TTransform<Scaler> inverseTransform(const TTransform<Scaler>& rhs) const
    {
        //Relative * this = rhs => Relative.translation + (Relative.rotation * this.translation) = rhs.translation
        //Relative * this = rhs => Relative.rotation * this.rotation = rhs.rotation
        const TQuaternion<Scaler> relativeRotation = rhs.rotation * rotation.getInverse();
        const TVector3<Scaler> relativeTranslation = rhs.translation - (relativeRotation * translation);
        return TTransform<Scaler>(relativeTranslation, relativeRotation);
    }

	TVector3<Scaler> translation;
	TQuaternion<Scaler> rotation;
};

typedef TTransform<float> Transform;

#endif
