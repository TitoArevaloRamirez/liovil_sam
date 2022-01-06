/*v1*************************************************************************************/
/* This is developed at Carnegie Mellon University in collaboration with Autel Robotics */
/*                                                                                      */
/* PI:                                                                                  */
/* George Kantor                                                                        */
/*                                                                                      */
/* Authors:                                                                             */ 
/* Weizhao Shao                                                                         */
/* Cong Li                                                                              */
/* Srinivasan Vijayarangan                                                              */
/*                                                                                      */
/* Please refer to the contract document for details on license/copyright information.  */
/****************************************************************************************/
std::vector<cv::KeyPoint> nms(const std::vector<cv::KeyPoint>& srcKeyPtrs) {
    std::vector<cv::Point2f> srcPtrs;
    std::vector<float> scores;
    std::vector<cv::Point2f> resPtrs;
    std::vector<cv::KeyPoint> resKeyPtrs;
    for (auto it=srcKeyPtrs.begin(); it!=srcKeyPtrs.end(); it++) {
        cv::Point2f pts(it->pt.x, it->pt.y);
        float resp(it->response);
        srcPtrs.push_back(pts);
        scores.push_back(resp);
    }
    // parameter
    float nbrSz = 10.0; // 2*nbr+1 x 2*nbr+1. centered at srcPtrs 
    float thresh = 1.f;
    int neighbors = 0;
    float minScoresSum = 0.f;
    // obtain size
    const size_t size = srcPtrs.size();
    if (!size) {
        std::cout << "nms size 0\n";
        return resKeyPtrs;
    }
    // Sort the bounding boxes by the detection score
    std::multimap<float, size_t> idxs;
    for (size_t i = 0; i < size; ++i)
        idxs.insert(std::pair<float, size_t>(scores[i], i));

    while (idxs.size() > 0) {
        // grab the last rectangle
        auto lastElem = --std::end(idxs);
        const cv::Point2f ptr1 = srcPtrs[lastElem->second];
        const cv::Rect& rect1 = cv::Rect(ptr1.x-nbrSz, ptr1.y-nbrSz, 2*nbrSz+1, 2*nbrSz+1);

        int neigborsCount = 0;
        float scoresSum = lastElem->first;

        idxs.erase(lastElem);

        for (auto pos = std::begin(idxs); pos != std::end(idxs); ) {
            // grab the current rectangle
            const cv::Point2f ptr2 = srcPtrs[pos->second];
            const cv::Rect& rect2 = cv::Rect(ptr2.x-nbrSz, ptr2.y-nbrSz, 2*nbrSz+1, 2*nbrSz+1);

            float intArea = (rect1 & rect2).area();
            float unionArea = rect1.area() + rect2.area() - intArea;
            float overlap = intArea / unionArea;

            //std::cout << "overlap area: " << overlap << std::endl;
            // if there is sufficient overlap, suppress the current bounding box
            if (overlap > thresh) {
                scoresSum += pos->first;
                pos = idxs.erase(pos);
                ++neigborsCount;
            }
            else
                ++pos;
        }
        // before push, check two other thing. now always true
        if (neigborsCount >= neighbors && scoresSum >= minScoresSum) {
            cv::KeyPoint tempKP = srcKeyPtrs[lastElem->second];
            resKeyPtrs.push_back(tempKP);
        }
    }
    return resKeyPtrs;
}
/*
void nms( const std::vector<cv::Point2f>& srcPtrs,
          const std::vector<float>& scores,
          std::vector<cv::Point2f>& resPtrs ) {
    // parameter
    float nbrSz = 3; // 2*nbr+1 x 2*nbr+1. centered at srcPtrs 
    float thresh = 1.f;
    int neighbors = 0;
    float minScoresSum = 0.f;
    // clear output container
    resPtrs.clear();
    // obtain size
    const size_t size = srcPtrs.size();
    if (!size)
        return;
    assert(srcPtrs.size() == scores.size());
    // Sort the bounding boxes by the detection score
    std::multimap<float, size_t> idxs;
    for (size_t i = 0; i < size; ++i)
        idxs.insert(std::pair<float, size_t>(scores[i], i));
    // keep looping while some indexes still remain in the indexes list
    while (idxs.size() > 0) {
        // grab the last rectangle
        auto lastElem = --std::end(idxs);
        const cv::Point2f ptr1 = srcPtrs[lastElem->second];
        const cv::Rect& rect1 = cv::Rect(ptr1.x-nbrSz, ptr1.y-nbrSz, 2*nbrSz+1, 2*nbrSz+1);

        int neigborsCount = 0;
        float scoresSum = lastElem->first;

        idxs.erase(lastElem);

        for (auto pos = std::begin(idxs); pos != std::end(idxs); ) {
            // grab the current rectangle
            const cv::Point2f ptr2 = srcPtrs[pos->second];
            const cv::Rect& rect2 = cv::Rect(ptr2.x-nbrSz, ptr2.y-nbrSz, 2*nbrSz+1, 2*nbrSz+1);

            float intArea = (rect1 & rect2).area();
            float unionArea = rect1.area() + rect2.area() - intArea;
            float overlap = intArea / unionArea;

            // if there is sufficient overlap, suppress the current bounding box
            if (overlap > thresh) {
                scoresSum += pos->first;
                pos = idxs.erase(pos);
                ++neigborsCount;
            }
            else
                ++pos;
        }
        // before push, check two other thing. now always true
        if (neigborsCount >= neighbors && scoresSum >= minScoresSum)
            resPtrs.push_back(ptr1);
    }
}
*/