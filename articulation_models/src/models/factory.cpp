/*
 * factory.cpp
 *
 *  Created on: Oct 22, 2009
 *      Author: sturm
 */

#include "articulation_models/models/factory.h"

#include <typeinfo>

#include "articulation_models/models/generic_model.h"
#include "articulation_models/models/rigid_model.h"
#include "articulation_models/models/prismatic_model.h"
#include "articulation_models/models/rotational_model.h"
#include "articulation_models/models/pca_gp_model.h"

using namespace std;

namespace articulation_models {

MultiModelFactory MultiModelFactory::instance;

MultiModelFactory::MultiModelFactory() {
	all_factories.push_back(new SingleModelFactory<RigidModel>("rigid"));
	all_factories.push_back(new SingleModelFactory<PrismaticModel>("prismatic"));
	all_factories.push_back(new SingleModelFactory<RotationalModel>("rotational"));
	all_factories.push_back(new SingleModelFactory<PCAGPModel>("pca_gp"));
	setFilter("");
}

GenericModelVector MultiModelFactory::createModels(const articulation_msgs::TrackMsg& trackMsg) {
	GenericModelVector models;
	for(size_t i=0;i<factories.size();i++) {
		models.push_back( factories[i]->createModel(trackMsg) );
	}
	return(models);
}

GenericModelVector MultiModelFactory::createModels(articulation_msgs::TrackMsgConstPtr trackMsg) {
	GenericModelVector models;
	for(size_t i=0;i<factories.size();i++) {
		models.push_back( factories[i]->createModel(trackMsg) );
	}
	return(models);
}

GenericModelVector MultiModelFactory::createModels(const articulation_msgs::ModelMsg& modelMsg) {
	GenericModelVector models;
	for(size_t i=0;i<factories.size();i++) {
		models.push_back( factories[i]->createModel(modelMsg) );
	}
	return(models);
}

GenericModelVector MultiModelFactory::createModels(articulation_msgs::ModelMsgConstPtr modelMsg) {
	GenericModelVector models;
	for(size_t i=0;i<factories.size();i++) {
		models.push_back( factories[i]->createModel(modelMsg) );
	}
	return(models);
}

GenericModelPtr MultiModelFactory::restoreModel(articulation_msgs::ModelMsgConstPtr modelMsg) {
	for(size_t i=0;i<factories.size();i++) {
		if( factories[i]->getLongName() == modelMsg->name ) {
			return( factories[i]->createModel(modelMsg) );
		}
	}
	return( GenericModelPtr() );
}

GenericModelPtr MultiModelFactory::restoreModel(const articulation_msgs::ModelMsg &modelMsg) {
	for(size_t i=0;i<factories.size();i++) {
		if( factories[i]->getLongName() == modelMsg.name ) {
			return( factories[i]->createModel(modelMsg) );
		}
	}
	return( GenericModelPtr() );
}

void MultiModelFactory::listModelFactories() {
	cout << "MultiModelFactory::listModelFactories(), "<< factories.size()<<" model factories registered:"<< endl;
	for(size_t i=0;i<factories.size();i++) {
		cout << factories[i]->getLongName() << endl;
	}
}

std::string MultiModelFactory::getLongName(GenericModel* model) {
	std::string classname = typeid(*model).name();
	for(size_t i=0;i<instance.factories.size();i++) {
		if( classname == instance.factories[i]->getClassName() )
			return instance.factories[i]->getLongName();
	}
	return("unknown_model");
}

int MultiModelFactory::getModelIndex(std::string name) {
	for(size_t i=0;i<instance.factories.size();i++) {
		if( name == instance.factories[i]->getClassName() || name == instance.factories[i]->getLongName() )
			return i;
	}
	return -1;
}

int MultiModelFactory::getFactoryCount() {
	return(factories.size());
}

void MultiModelFactory::setFilter(std::string filter) {
	if(filter=="") {
		factories = all_factories;	// allow all
//		cout << "MultiModelFactory: resetting filter"<<endl;
		for(size_t i=0;i<factories.size();i++) {
//			cout << "MultiModelFactory: adding "<< factories[i]->getLongName()<< endl;

		}
		return;
	}
//	cout << "MultiModelFactory: setting filter to '" << filter<<"'"<<endl;
	std::stringstream ss(filter);
	factories.clear();
	while(!ss.eof()) {
		std::string allowed_model = "";
		ss >> allowed_model;
		bool found = false;
		for(size_t i=0;i<all_factories.size();i++) {
			if( all_factories[i]->getLongName() == allowed_model ) {
				factories.push_back(all_factories[i]);
//				cout << "MultiModelFactory: adding "<< allowed_model<< endl;
				found = true;
			}
		}
		if(!found) {
			cout << "MultiModelFactory: cannot add, because model unknown: "<< allowed_model<< endl;
			exit(1);
		}
	}
}


}
