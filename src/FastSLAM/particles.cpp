/**
 * @file particles.cpp
 * @brief implements the FastSLAM 2D particles class
 */

#include "particle-filter.h"

FastSLAMParticles::FastSLAMParticles(const FastSLAMParticles& part):
    m_importance_factor(part.m_importance_factor),
    m_robot_pose(part.m_robot_pose),
    m_data_label(part.m_data_label),
    m_robot(part.m_robot){
    for (const auto& it: m_lmekf_bank){
        m_lmekf_bank.push_back(std::make_pair(std::make_unique<LMEKF2D>(*it.first.get()), it.second));
    }
}

int FastSLAMParticles::matchLandmark(const struct Observation2D& curr_obs) {
    float w_0 = this->m_importance_factor;
    int landmark_id = m_lmekf_bank.size();
    int idx = 0;

    for (auto const& it: m_lmekf_bank) {
        it.first->updateObservation(curr_obs);
        float w_n = it.first->calcCPD();
        landmark_id = w_n > w_0 ? idx : landmark_id;
        idx++;
    }

    m_data_label = landmark_id;

    return landmark_id;
}

PF_RET FastSLAMParticles::updateLMBelief(const struct Observation2D& curr_obs){
    if (m_robot == nullptr) {
        // appropriate error handling here
        std::cout << "non-robot manager specified" << std::endl;
        return PF_RET::EMPTY_ROBOT_MANAGER;
    }
    if (m_data_label == m_lmekf_bank.size()) {
        // initiate new EKF
        struct Point2D proposed_mean = m_robot->inverseMeas(m_robot_pose, curr_obs);
        auto meas_jacobian = m_robot->measJacobian(proposed_mean);

        Eigen::Matrix2f proposed_cov;
        if (meas_jacobian.determinant() == 0) {
            // log error here
            std::cout << "Non-invertible matrix" << std::endl;
            proposed_cov = Eigen::Matrix2f::Identity();
        } else {
            proposed_cov = meas_jacobian.inverse() * m_robot->getMeasNoise() *
                meas_jacobian.inverse().transpose();
        }

        std::unique_ptr<LMEKF2D> new_lmefk =
            std::make_unique<LMEKF2D>(proposed_mean, proposed_cov, m_robot);
        m_lmekf_bank.push_back(std::make_pair(std::move(new_lmefk), 1));
        return PF_RET::SUCCESS;
    } else {
        LMEKF2D * filter_to_update = m_lmekf_bank[m_data_label].first.get();
        filter_to_update->updateObservation(curr_obs);
        auto status = filter_to_update->update();

        switch (status) {
            case KF_RET::EMPTY_ROBOT_MANAGER:
                // error handling
                std::cout << "non-robot manager specified" << std::endl;
                return PF_RET::EMPTY_ROBOT_MANAGER;
            case KF_RET::MATRIX_INVERSION_ERROR:
                // error handling
                std::cout << "Kalman Filter failed to converge" << std::endl;
                return PF_RET::MATRIX_INVERSION_ERROR;
            default:
                m_lmekf_bank[m_data_label].second++;
                break;
        }
    }
    return PF_RET::SUCCESS;
}

#ifdef LM_CLEANUP
void FastSLAMParticles::cleanUpSightings() {
    int i = 0;
    for (auto& it: m_lmekf_bank) {
        if (i == m_data_label) {
            i++;
            continue;
        }

        if ( MathUtil::findDist(it.first->getLMEst(), m_robot_pose) <=
             m_robot->getPerceptualRange() ) {
            it.second--;
        }
    }
}
#endif

PF_RET FastSLAMParticles::updatePose(const struct Pose2D& new_pose) {
    m_robot_pose = new_pose;
    return PF_RET::SUCCESS;
}

float FastSLAMParticles::updateParticle(const struct Observation2D& new_obs,
                                      const struct Pose2D& new_pose) {
    if (m_robot == nullptr) {
        std::cout << "No robot manager specified" << std::endl;
        return -1.0;
    }
    int res_code = 0;
    res_code += static_cast<int>(updatePose(new_pose));
    matchLandmark(new_obs);
    res_code += static_cast<int>(updateLMBelief(new_obs));

#ifdef LM_CLEANUP
    cleanUpSightings();
#endif //LM_CLEANUP

    return res_code == static_cast<int>(PF_RET::SUCCESS) ?
        m_lmekf_bank[m_data_label].first->calcCPD() :
        static_cast<float>(PF_RET::UPDATE_ERROR);
}
