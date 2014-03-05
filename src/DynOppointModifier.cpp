/*
 * Copyright (c) 2013, PAL Robotics, S.L.
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

/** \author Karsten Knese
 */

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <dynamic-graph/factory.h>
#include <dynamic-graph/all-signals.h>
#include <dynamic-graph/all-commands.h>

#include <sot-core/DynOppointModifier.hh>

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

using namespace std;
using namespace dynamicgraph::sot;
using namespace dynamicgraph;


/* --- DG FACTORY ------------------------------------------------------- */
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(DynamicOpPointModifier,"DynamicOppointModifier");

/* ---------------------------------------------------------------------- */
/* --- CONSTRUCTION ----------------------------------------------------- */
/* ---------------------------------------------------------------------- */


DynamicOpPointModifier::DynamicOpPointModifier( const std::string & name )
    : OpPointModifier(name)
    ,CONSTRUCT_SIGNAL_IN(transformationSig,dynamicgraph::Vector)
{
    jacobianSOUT.setFunction( boost::bind(&DynamicOpPointModifier::jacobianSOUT_function,this,_1,_2) );
    positionSOUT.setFunction( boost::bind(&DynamicOpPointModifier::positionSOUT_function,this,_1,_2) );
    // Commands
    std::string docstring;

    docstring =
            "\n"
            " Child of OpPointModifier in order to listen to transformation as a signal."
            "\n";

    signalRegistration(transformationSigSIN);
}

sot::MatrixHomogeneous DynamicOpPointModifier::convertToHomo(const Vector &translation) const
{
    sot::MatrixHomogeneous transformation;
    transformation.setIdentity();
    transformation.elementAt(0,3) = translation.elementAt(0);
    transformation.elementAt(1,3) = translation.elementAt(1);
    transformation.elementAt(2,3) = translation.elementAt(2);

    return transformation;
}

ml::Matrix&
DynamicOpPointModifier::jacobianSOUT_function( ml::Matrix& res,const int& iter )
{
    const dynamicgraph::Vector& translation = transformationSigSIN(iter);
    sot::MatrixHomogeneous transformation= convertToHomo(translation);

    OpPointModifier::setTransformation(transformation);
    OpPointModifier::jacobianSOUT_function(res,iter);
    return res;
}

MatrixHomogeneous&
DynamicOpPointModifier::positionSOUT_function( MatrixHomogeneous& res,const int& iter )
{
    const dynamicgraph::Vector& translation = transformationSigSIN(iter);
    sot::MatrixHomogeneous transformation = convertToHomo(translation);

//    std::cout << "DYNOP: used transformation :" << std::endl;
//    std::cout << transformation << std::endl;
    OpPointModifier::setTransformation(transformation);
    OpPointModifier::positionSOUT_function(res,iter);
//    std::cout << "DYNOP: position after multiplying"<<iter  << std::endl;
//    std::cout << res << std::endl;
    return res;
}
