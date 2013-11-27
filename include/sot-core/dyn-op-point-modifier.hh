/*
 * Copyright 2011, Nicolas Mansard, LAAS-CNRS
 *
 * This file is part of sot-dyninv.
 * sot-dyninv is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-dyninv is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-dyninv.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __sot_DynamicOpPointModifier_H__
#define __sot_DynamicOpPointModifier_H__

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* SOT */
#include <sot/core/op-point-modifier.hh>
#include <sot-dyninv/signal-helper.h>
#include <sot-dyninv/entity-helper.h>
#include <sot/core/task.hh>
#include <sot/core/flags.hh>

namespace dynamicgraph {
namespace sot {
namespace dg = dynamicgraph;

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

class DynamicOpPointModifier
        :public OpPointModifier
{

public: /* --- CONSTRUCTOR ---- */
    static const std::string CLASS_NAME;
    DynamicOpPointModifier( const std::string& name );

public:  /* --- SIGNALS --- */
    sot::MatrixHomogeneous& setTransformationSig(sot::MatrixHomogeneous& res, int time );
    ml::Matrix& jacobianSOUT_function( ml::Matrix& res,const int& iter );
    MatrixHomogeneous& positionSOUT_function( MatrixHomogeneous& res,const int& iter);
    DECLARE_SIGNAL_IN(transformationSig,sot::MatrixHomogeneous);

}; // class TaskVelocityDamping

} // namespace sot
} // namespace dynamicgraph


#endif // #ifndef __sot_TaskVelocityDamping_H__

