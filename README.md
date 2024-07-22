# Contains source code for CV2X
Still work in progress.

Primarily using python interface if possible. C and python interfaces shares the same functionalities.

Please refer to group project folder [Commsignia](https://buffalo.box.com/s/f4c25fn1orqedz6q7veceno4writetx1) for details on how to setup Commsignia APIs. The current sdk version in use is `Unplugged-RT-y20.39.4-b205116-pythonsdk.tar.xz`.

### Protocol on testing the codes

All codes need to be tested before merging.

To facilitate the test procedure, the following protocol will be enforced. The goal of this protocol is to make sure any code changes during the test be recorded properly and can be retrieve in the future.

1. Maintain a fork of this repo under your account (your origin). The repo in CHELabUB 

2. Push a test branch with all the testing code and script that are relevant to this repo to your own fork. The branch shall be name with `test_yyyymmdd_xxx` where `_xxx` are required unless there is no ambiguity, it can be word(s) used to describe test purpose.

3. Submit a PR to the upstream branch, mark it with label "test" describe briefly the test procedure and expected outcomes. 

4. Once the test is concluded, briefly summarize the results and then close the PR.

Remark 1: Reuse the same branch for different tests are allowed (though not encouraged), as long as the commits can clearly indicate the changes across different tests.

Remark 2: When merging the code, you need to create a designated PR and point towards the test PRs as a proof.
