# Road Runner Quickstart

Check out the [docs](https://rr.brott.dev/docs/v1-0/tuning/).

# To Get Latest goBILDA Odometry Computer Files

Run this once to setup a remote URL called `pinpoint`:

`git remote add pinpoint https://github.com/goBILDA-Official/FtcRobotController-Add-Pinpoint`

Then run these commands to merge in the latest changes
```bash
git checkout competition
git fetch pinpoint
git merge pinpoint/goBILDA-Odometry-Driver
```

or

`git checkout -p pinpoint/goBILDA-Odometry-Driver .\TeamCode\src\main\java\org\firstinspires\ftc\teamcode\GoBildaPinpointDriver.java` (for a single file)

(`git merge --abort` is your friend is something isn't right)

Then commit it as usual.

If there is a conflict with README.md, just revert back to ours:
```bash
git reset HEAD README.md
git restore README.md
```