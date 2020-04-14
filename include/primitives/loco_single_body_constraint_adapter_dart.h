#pragma once

#include <loco_common_dart.h>
#include <primitives/loco_single_body_constraint_adapter.h>

namespace loco {
    class TISingleBodyConstraint;
}

namespace loco {
namespace dartsim {

    class TIDartSingleBodyConstraintAdapter
    {
    public :
        TIDartSingleBodyConstraintAdapter();

        virtual ~TIDartSingleBodyConstraintAdapter();

        void SetDartSkeleton( dart::dynamics::Skeleton* skeleton_ref ) { m_DartSkeletonRef = skeleton_ref; }

        dart::dynamics::Skeleton* skeleton() { return m_DartSkeletonRef; }

        const dart::dynamics::Skeleton* skeleton() const { return m_DartSkeletonRef; }

        dart::dynamics::Joint* joint() { return m_DartJointRef; }

        const dart::dynamics::Joint* joint() const { return m_DartJointRef; }

        dart::dynamics::BodyNode* body_node() { return m_DartBodyNodeRef; }

        const dart::dynamics::BodyNode* body_node() const { return m_DartBodyNodeRef; }

    protected :

        // Reference to skeleton-ptr (to create joint|body-node aspect)
        dart::dynamics::Skeleton* m_DartSkeletonRef;
        // Reference to joint-ptr created for this constraint
        dart::dynamics::Joint* m_DartJointRef;
        // Reference to bodynode-ptr created during constraint creation
        dart::dynamics::BodyNode* m_DartBodyNodeRef;
    };

    class TDartSingleBodyRevoluteConstraintAdapter : public TISingleBodyRevoluteConstraintAdapter,
                                                     public TIDartSingleBodyConstraintAdapter
    {
    public :

        TDartSingleBodyRevoluteConstraintAdapter( TISingleBodyConstraint* constraint_ref )
            : TISingleBodyRevoluteConstraintAdapter( constraint_ref ), TIDartSingleBodyConstraintAdapter() {};

        TDartSingleBodyRevoluteConstraintAdapter( const TDartSingleBodyRevoluteConstraintAdapter& other ) = delete;

        TDartSingleBodyRevoluteConstraintAdapter& operator= ( const TDartSingleBodyRevoluteConstraintAdapter& other ) = delete;

        ~TDartSingleBodyRevoluteConstraintAdapter() = default;

        void Build() override;

        void Initialize() override;

        void Reset() override;

        void SetHingeAngle( TScalar hinge_angle ) override;

        void SetLimits( const TVec2& limits ) override;

        void GetHingeAngle( TScalar& dst_hinge_angle ) override;

    private :

        void _SetHingeSpeed( TScalar hinge_speed );
    };

    class TDartSingleBodyPrismaticConstraintAdapter : public TISingleBodyPrismaticConstraintAdapter,
                                                      public TIDartSingleBodyConstraintAdapter
    {
    public :

        TDartSingleBodyPrismaticConstraintAdapter( TISingleBodyConstraint* constraint_ref )
            : TISingleBodyPrismaticConstraintAdapter( constraint_ref ), TIDartSingleBodyConstraintAdapter() {}

        TDartSingleBodyPrismaticConstraintAdapter( const TDartSingleBodyPrismaticConstraintAdapter& other ) = delete;

        TDartSingleBodyPrismaticConstraintAdapter& operator= ( const TDartSingleBodyPrismaticConstraintAdapter& other ) = delete;

        ~TDartSingleBodyPrismaticConstraintAdapter() = default;

        void Build() override;

        void Initialize() override;

        void Reset() override;

        void SetSlidePosition( TScalar slide_position ) override;

        void SetLimits( const TVec2& limits ) override;

        void GetSlidePosition( TScalar& dst_slide_position ) override;

    private :

        void _SetSlideSpeed( TScalar slide_speed );
    };

}}