# CV2X based vehicle planning and control
This repo contains source codes for CV2X based vehicle control.

The codes are based on Commsignia V2X Remote Python SDK. The SDK is a python package to access low level functionality of the Commsignia V2X stack. 

**Any code in this repository that uses Commsignia APIs, including links to data and documents, requires a valid license and IP protected. You may not share or use this code without proper authorization from PI Prof. Chaozhe He. If you have any questions or need access, please email Prof. He at chaozheh@buffalo.edu.**

Disclaimer: The programs in this repo are for research purposes only and may not be fully functional without the appropriate Commsignia hardware and/or [UB-CAVAS](https://github.com/ub-cavas) connected automated vehicle setup.


### Protocol on testing the codes

All codes need to be tested before merging.

To facilitate the test procedure, the following protocol will be enforced. The goal of this protocol is to make sure any code changes during the test be recorded properly and can be retrieve in the future.

1. Maintain a fork of this repo under your account (your origin). The repo in CHELabUB 

2. Push a test branch with all the testing code and script that are relevant to this repo to your own fork. The branch shall be name with `test_yyyymmdd_xxx` where `_xxx` are required unless there is no ambiguity, it can be word(s) used to describe test purpose.

3. Submit a PR to the upstream branch, mark it with label "test" describe briefly the test procedure and expected outcomes. 

4. Once the test is concluded, briefly summarize the results and then close the PR.

Remark 1: Reuse the same branch for different tests are allowed (though not encouraged), as long as the commits can clearly indicate the changes across different tests.

Remark 2: When merging the code, a designated PR is needed with links towards the test PRs as a proof.
