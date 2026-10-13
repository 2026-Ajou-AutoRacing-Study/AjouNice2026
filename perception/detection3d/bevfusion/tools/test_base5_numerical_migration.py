import copy
import unittest
from migrate_base5_numerical_resume import check_identity


class MigrationTest(unittest.TestCase):
    def fixture(self):
        return dict(model={'channels':[16,32]},lr=1e-4,
                    source_hashes={'perception/detection3d/bevfusion/plugin/morai_rectangular_bev.py':'old',
                                   'unrelated.py':'unchanged'})

    def test_only_enumerated_source_change_and_tuple_json_equivalence(self):
        old=self.fixture();new=copy.deepcopy(old)
        new['model']['channels']=(16,32)
        new['source_hashes']['perception/detection3d/bevfusion/plugin/morai_rectangular_bev.py']='new'
        self.assertEqual(len(check_identity(old,new)),1)

    def test_lr_model_and_unrelated_code_changes_are_rejected(self):
        old=self.fixture()
        for key,value in [('lr',2e-4),('model',{'channels':[16,64]})]:
            new=copy.deepcopy(old);new[key]=value
            with self.assertRaisesRegex(ValueError,'training identity'):
                check_identity(old,new)
        new=copy.deepcopy(old);new['source_hashes']['unrelated.py']='changed'
        with self.assertRaisesRegex(ValueError,'Unapproved'):
            check_identity(old,new)


if __name__=='__main__':unittest.main()
